# Round 6: C++コアエンジン（交通シミュ本体）の効率化 — 調査とP1+P2実装

実施: 2026-07-03（調査→公式テスト追加→P1+P2本実装まで完了。実装内容は文末の「実装完了報告」参照）

## 計測環境

- シナリオ: 11x11グリッド，deltan=3, tmax=7200, no_cyclic_routing=True, 約19,844台（bench_heavy相当）
- OMP_NUM_THREADS=1，perfはparanoid=4で使用不可のためstd::chronoフェーズ計測（Round 2と同方式）
- マシンノイズが大きいため（run間±20%），A/B比較はインターリーブ実行（base→変種を交互）で実施

## 現状プロファイル

### main_loop内フェーズ内訳（中央値 約2.0s）

| フェーズ | 時間 | 割合 |
|---|---|---|
| veh_update | 1.46s | 73% |
| car_follow | 0.31s | 15% |
| node_transfer | 0.14s | 7% |
| link_update | 0.07s | 3% |
| node_generate他 | 0.03s | 2% |
| route_choice | 0.02s | 1% |

### 構造的な観測

1. **veh_updateループの反復47.6M回のうち実作業（RUN）は9.0M回（19%）**
   - END/ABORT済みスキップ: 26.5M回（56%）— ポインタ間接参照＋state判定のみの無駄走査
   - HOME（未出発）: 11.9M回（25%）— 出発時刻チェックのみ
   - WAIT: 0.13M回
   - car_followループ（active_vehicles走査21M回）も同様にRUNは9.0M回（43%）
2. **log_dataがmain_loopの約45%（≈0.8s）**: vehicle_logging_timestep_interval=0にするとveh_update 1.2→0.44s，main_loop全体1.8→0.78s
3. **exec_simulation全体（約3.4s）のうちC++ main_loopは約2.0s**。残りはPythonラッパー:
   - `_build_all_vehicle_log_caches`: **1.2s**（per-vehicleループ，numpy full/arange 2回/台 + concatenate 7回/台）
   - `_build_vehicles_enter_log`: 0.2s，adddemand: 0.37s（シナリオ構築）
4. ピークRSS約2.5GB（ログ配列が支配的）

## 検証済みプロトタイプ（P1, P2）

パッチ: `tmp/20260703_proto_p1_p2_core_optimization.patch`（`#ifdef P1_UPDATE_ORDER` / `#ifdef P2_LOG_SLIM` でA/B可能。計測用インストルメンテーション込みなので本実装時は要整理）

### P1: veh_update/car_followの死に走査除去

- World（プロトタイプではstatic）に`update_order`（HOME/WAIT/RUNのid順リスト）を保持。main_loopチャンク開始時に再構築し，毎stepの更新ループ内でin-place圧縮（END/ABORTになった車両を落とす）
- **id順が保存されるためRNG消費順・浮動小数点加算順が不変 → ビット同一**
- car_followループも同リストを走査（car_follow_newellは順序非依存，speed_sumは進捗表示のみ）
- hard_deterministic_mode分岐は従来のまま（元々全車両id順走査）

### P2: log_t / log_stateの記録廃止（エクスポート時再構成）

ログ列の構造性を利用: 各車両のログは常に `HOME×1, WAIT×w, RUN×r, (END|ABORT)×e` の順で，出発後は1エントリ/step。例外はトリップ終了だけで，update()経由の終了はENDを同一stepに2回記録（end_trip内+update内），transfer経由は1回。

- Vehicleにスカラー4個を追加: `log_first_ts, log_last_ts, log_wait_count, log_end_count`
- log_dataでのlog_t/log_state pushを廃止（12B/エントリ削減，6配列→4配列）
- エクスポート（flat builder / enter_log builder）で再構成:
  - `t_i = (double)(log_first_ts + i) * delta_t`（末尾e個は`log_last_ts`）— eager版と同一式でビット同一
  - `state_i`: 位置から決定（末尾e個はveh->state）
- **注意（プロトタイプ未対応分）**: bindings.cppの生`log_t`/`log_state`ゲッター（868行付近）も再構成に置き換える必要あり。チャンク実行途中のログ読み出し・DTAソルバー経由は未検証

### 計測結果（6ラウンドインターリーブ，中央値）

| 指標 | base | P1 | P1+P2 |
|---|---|---|---|
| main_loop | 2.006s | 1.793s (−11%) | **1.525s (−24%)** |
| veh_update | 1.46s | 1.28s | 1.03s |
| exec_simulation | 3.42s | 3.16s | **2.92s (−15%)** |
| ピークRSS | 2.47GB | — | **2.20GB (−266MB)** |

- 6ラウンド全てで base > P1 > P1+P2 の順序が一貫
- P1単体は別バッチ計測では−25%を観測した回もある（ノイズ大，効果量は10-25%の幅で見るべき）
- **ビット同一性**: 全車両ログ（t/x/v/state/lane/link）・travel_time/arrival_time・全リンクtraveltime_actual/instant・TTTのsha256が2seedでbaseと完全一致

## 未実装の追加候補（推定効果順）

1. **HOME車両の出発時刻バケット化**: P1後も残る死に走査の主因（11.9M回，P1後リストの57%）。finalize時にdeparture_timestepごとのバケットを作り，該当stepで初めてリストに載せる → veh_updateさらに−0.2s規模の見込み。Python側からの`departure_time`書き換え（bindingsでdef_rw）があるためチャンク境界での再構築が必要
2. **ラッパー`_build_all_vehicle_log_caches`（1.2s）**: exec_simulation最大の単一項目。C++側でhome-prepend込みのfullフラット配列を返しPython側のconcatenate/arange/fullを全廃する案（Round 4放棄案の再訪）。Round 4はlog-heavyベンチで+18%退行の履歴があるため**measure-first必須**だが，当時と違い今回は heavyベンチのプロファイルで1.2sと定量化済み
3. **残り4ログ配列のさらなる圧縮**: laneをint8化，またはAoSパック化（1 push_back/step）。Round 4のAoS放棄履歴あり，measure-first
4. 小粒（効果は各数十ms以下と推定）:
   - `Node::transfer`の`_buf_seen_outlinks`（unordered_set）→ 小さいvector線形走査
   - `Vehicle::route_preference`はエンジン内部・ラッパーとも未使用（World側を使用）なのに全車両で`links.size()`個のdoubleを確保（本ベンチで計70MB）。bindingsのdef_rwを保ったままlazy化可能
   - per-vehicleの`_buf_outlinks`等3ベクトル → World共有スクラッチ化
   - ログreserveの過大（total_timesteps+10/台 → 本ベンチで仮想1.7GB確保，実使用44%）

## その他の気づき（コード衛生・要修正ではない）

- main_loopのvehicle updateループ: hard_deterministic分岐と非分岐が完全に同一コード
- `build_enter_log_data`内のn_missing計算は結果未使用（デッドコード）
- `uxsim/trafficpp/build/`のCMakeCacheが別ソースツリー（`/home/toruseo/test-UXsim-UXsimpp-integration`）を指しており，このディレクトリでビルドしても本リポジトリのソースは反映されない。正規手順は`pip install -e .`（本環境では`PIP_BREAK_SYSTEM_PACKAGES=1`が必要）

## チャンク実行＋途中介入下での検証（追加実施）

`exec_simulation(duration_t2=600)`で12チャンクに分割し，**毎境界**で以下を実施するシナリオでbaseとP1+P2を比較:

- 車両ログの読み出し（走行中RUN・待機WAIT・未出発HOME・終了済みENDの各状態を含む5台をサンプル，`log_t/x/v/state/lane/link` + `log_t_link`）※毎回`_log_cache`をリセットして再構成経路を強制
- リンク`traveltime_actual`（遅延materialize経路）・`traveltime_instant`の読み出し
- VEHICLES_LIVING/RUNNING件数
- 介入: 境界2で`adddemand`（シミュ途中の車両追加→P1のリスト再構築経路），境界3で`change_free_flow_speed`

**結果: 12境界すべてのチェックサム＋最終全ログチェックサム＋TTTがbaseとP1+P2で完全一致**（`tmp/`の`chunked_base.txt`/`chunked_p1p2.txt`）

- P1が介入に耐える理由: `update_order`はmain_loop呼び出しごとに`vehicles`（id順）から再構築するため，チャンク間の車両追加・状態変更を必ず拾う
- P2が介入に耐える理由: 再構成はスカラー4個からのステートレス計算で，走行中車両（ENDテールなし）も位置規則だけで正しく復元される

### 検証で発見した既存バグ（P1/P2と無関係，現行HEADで再現）

**C++モードで途中に`veh.log_t`等を読むと，全車両の`_log_cache`が構築されたまま二度と無効化されず，最終ログが凍結される**。

- 再現: 3ノード直列，`exec_simulation(duration_t2=50)`→`VEHICLES['0'].log_t`読み出し→完走。Pythonモードはlog長10→25と成長，C++モードは11→11で凍結（stateは正しくendになる）
- 原因: `_log_cache`のNone化は`_register_new_cpp_vehicles`のみ。`_sync_from_cpp`のdocstringに「Log caches are never explicitly invalidated」と明記されており，途中読み出しを想定していない設計
- 影響: 1台でも途中でログを読むと全車両分が凍結（batch構築のため）。analyzerの最終結果も汚染される
- 修正案: `exec_simulation`（または`_sync_from_cpp`）で全車両の`_log_cache`をNoneに戻す。ただし途中読み出しのたびに全車両分を再構築するコストとのトレードオフに注意
- 付随観察: 途中時点でのログ長がPython 10 vs C++ 11と1エントリずれる（duration_t2の境界セマンティクス起因とみられる．最終結果は一致）

## 検証状況

- 調査後，リポジトリのソースはHEADに復帰し，`pip install -e .`でクリーン再インストール済み
- 復元後の結果チェックサムが調査前ベースラインと一致，`test_cpp_mode.py`インラインテスト174件通過（4 rerun，既知flaky）を確認

## 実装完了報告（同日）

調査後，以下の順で実装（コーディングはOpusサブエージェント，設計・レビュー・検証はメイン）：

### 1. 公式テスト追加（先行）

`tests/test_cpp_mode.py` に `test_chunked_exec_with_midsim_reads_and_interventions` を追加（P1+P2適用**前**のHEADエンジンで通過を確認してから実装を適用）。

- 読み出しあり/なしの2チャンク実行ランを比較し，境界での読み出し（車両ログ・リンク旅行時間）が最終結果に副作用を与えないことを完全一致で検証
- 途中読み出しログの構造不変条件（配列長一致，DELTAT刻み単調増加＋末尾ゼロ差分のみ許容，状態遷移の非減少列）を検証
- 既知のログキャッシュ凍結制約は `_log_cache=None` リセットで回避（コメント付き）
- 実行時間約3.5s。テスト追加後の収集数200→201
- テスト作成時の発見: トリップ終了時の同一timestepエントリは実際は**3個**（RUN×1＋END×2，update()経由）。調査時devlogの「2回」はENDエントリのみの数で，log_t差分としては末尾ゼロが2個並ぶ

### 2. P1+P2本実装

変更: `traffi.h`, `traffi.cpp`, `bindings.cpp`(+10), `dta_solver.cpp`(+1-1)

- **P1**: `World::update_order`（id順HOME/WAIT/RUN，main_loop呼び出しごとに再構築，毎stepインプレース圧縮）。car_follow/veh_updateのhard_deterministic分岐を単一ループに統合（update_orderはid順なので意味論不変．veh_update側の両分岐は元々同一コードだった）。孤立した`World::active_vehicles`と`Vehicle::active_index`は全参照ごと削除
- **P2**: `Vehicle::log_t`/`log_state`ベクトルを削除し，スカラー4個（`log_first_ts`/`log_last_ts`/`log_wait_count`/`log_end_count`）＋再構成メソッド`log_t_at(i)`/`log_state_at(i)`に置換。消費側の移植: flat builder（n_missing/本体/ltl），`build_enter_log_data`（未使用だったn_missing計算は削除），bindingsの生`log_t`/`log_state`ゲッター，`dta_solver.cpp`の`dta_get_traveled_route`

### 3. 検証結果

- **ビット同一性**: 単発実行（2seed，全車両ログ・traveltime・TTTのsha256）＝調査時ベースラインと完全一致。チャンク+介入実行（12境界チェックサム+最終）＝完全一致
- **テスト**: `test_cpp_mode.py` **201件全通過**（--reruns 5，2 rerun既知flaky，6分30秒）
- **確認ベンチ**（クリーンビルド同士，インターリーブ5ペア，11x11グリッド）: exec_simulation中央値 3.17s→2.81s（**−11.5%**，4勝1分）。main_loop単体の−24%（調査時計測）はPython後処理約1.4sで希釈される。exec全体の改善幅は調査時の−15%と整合
- コミットは未実施（指示待ち）

## 追補: トリップ終了ログのPython仕様不一致の発見と修正（同日）

公式テスト作成時に発見された「トリップ終了時に同一timestepへ3エントリ」の齟齬を精査した結果，**C++側の既存バグ**と判明（Round 6以前から存在，P1/P2とは無関係）:

- **正（Python, uxsim.py）**: update()冒頭の`record_log()`（RUN@k）＋`end_trip()`内の`record_log(enforce_log=1)`（END@k）のみ。update()経由の終了で最終timestepに**2エントリ**，transfer経由なら**1エントリ**
- **C++のバグ**: `Vehicle::update()`のdest到達分岐とdead-end abort分岐が`end_trip(); log_data();`となっており，ENDを二重記録（3エントリ，配列長も車両あたり+1）
- **修正**: 余分な`log_data()`2箇所を削除（traffi.cpp）。P2の再構成は`log_end_count`で一般化済みのためロジック変更不要（修正後はlog_end_count≦1）。コメントを実態に合わせ更新
- **検証**: Python vs C++の全車両ログ厳密比較（渋滞コリドー150台＋abortコリドー300台，全ノード出次数≦1で乱数非依存の決定的シナリオ）——修正前は全車両で+1エントリのFAIL，修正後は**log_x/log_vも含め完全一致（==）でPASS**。`test_cpp_mode.py`全201件通過
- **注意**: この修正によりC++のログ出力が（正しい方向に）変わるため，Round 6調査時のsha256ベースラインは無効。以後の等価性の正はPython版
- 公式テストとして `test_vehicle_log_python_cpp_equivalence` を追加（Python↔C++全車両ログ厳密一致，トリップ終了エントリ構造の回帰テスト）
