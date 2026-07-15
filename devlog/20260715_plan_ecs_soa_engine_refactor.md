# 計画: C++エンジン内部のECS/SoA化リファクタ

作成: 2026-07-15（同日改訂: 乱数消費順序の保存要件を撤廃し設計を効率化）
ブランチ: `refactor/ecs-soa-engine`
ステータス: 計画（未着手）

## 1. 目的

C++エンジン（`uxsim/trafficpp/traffi.h`, `traffi.cpp`）の内部データ構造をデータ指向（SoA: Structure of Arrays）に抜本的に再設計し，シミュレーション本体を高速化する。
外部から見える機能・インターフェース（`World(cpp=True)` のPython互換API，bindings，ラッパー，analyzer連携，DTAソルバー）は一切変えない。
これを実現するため，SoAコアの上に既存のアクセス方法を提供するC++ファサード層を設け，bindings.cpp・uxsim_cpp_wrapper.py の変更を最小限に抑える。

**等価性の要件（ユーザー指定）**: 乱数消費順序を保つ必要はない。
乱数の影響を受けないシナリオではビット同一を維持し，確率的シナリオでは30回程度の試行結果が統計的に同等であればよい。

**スコープ（ユーザー指定）**: 対象はコアシミュレータのみ。
DUE/DSOソルバーはコアが高速化されれば自動的に効率化されるため，専用のベンチ・統計検証は行わない（`dta_solver.cpp` はファサードへの機械的追従でコンパイル・動作を維持し，既存リグレッションテストで担保する）。

## 2. 現状アーキテクチャの分析

### 2.1 データ構造（AoS）

- `World` がエンティティレジストリ（`vector<Vehicle*>`, `vector<Link*>`, `vector<Node*>`）とグローバル配列（`route_preference`, `adj_mat_time`, `rsa_*` スクラッチ）を保持
- `Node` / `Link` は少数（典型的に数十〜数百個）のヒープオブジェクト。リンクの時系列（`arrival_curve` 等）は既に連続配列であり，キャッシュ効率の問題は小さい
- `Vehicle` は多数（数万〜数十万プラトーン）の個別ヒープオブジェクト。ホット変数（`x`, `x_next`, `v`, `move_remain`, `state`, `link`, `leader`）とコールドデータ（`name`, ログ5配列，`route_preference` O(L)，`_traveled_nodes` O(N)，バッファ3本）が混在し，1台あたり数百バイト＋可変長ベクトル群
- リンク上の車両キューは `deque<Vehicle*>`。leader/follower はポインタで相互リンクし，generate/transfer/end_trip の3箇所で繋ぎ替え管理
- メインループは全車両をid順の `update_order` で走査（HOME/WAIT/RUN混在，END/ABORTは圧縮除去済み）

### 2.2 プロファイル（Round 6, 2026-07-03, 11x11グリッド heavy bench）

P1（update_order）+P2（log_t/log_stateスカラー化）適用後の構成が現在のHEAD。

| 項目 | 時間 | 備考 |
|---|---|---|
| exec_simulation全体 | 2.81s | |
| ├ C++ main_loop | 約1.5s | |
| │ ├ veh_update | 約1.0s | 大半がlog_data（5配列へのpush_back）。HOME車両の出発時刻チェック走査も残存（P1後リストの57%） |
| │ ├ car_follow | 約0.3s | leader->x のランダムアクセス |
| │ ├ node_transfer | 約0.14s | |
| │ └ link_update他 | 約0.1s | |
| └ Pythonラッパー後処理 | 約1.3s | `_build_all_vehicle_log_caches` 等 |

- 対Pythonのスピードアップ実績: 15.2x（9x9）〜31.5x（信号8x8）
- ピークRSS約2.2GB（ログ配列とper-vehicle固定費が支配的）

### 2.3 AoS構造に起因する非効率（本リファクタの対象）

1. **log_dataのライトストリーム分散**: 毎step毎車両，車両ごとに別ヒープ領域にある5本のvectorへpush_back。書き込み先がRUN車両数ぶん散在する
2. **car_followのポインタ間接**: `update_order`（id順）で走査するため，`veh->x` も `leader->x` もヒープ上に散在。リンク内の車列は本来規則的（leaderはリンクキューの `number_of_lanes` 個前）なのに，その構造を活かせていない
3. **状態混在の走査**: HOME/WAIT/RUNが同一リストに混在し，HOME車両は出発時刻チェックのためだけに毎step走査される。従来は乱数消費順序（id順）を保つためにこの構造が必要だったが，今回この制約は撤廃された
4. **per-vehicle固定費**: `route_preference`（O(L)，エンジン内部では未使用），`_traveled_nodes`（O(N)），選択バッファ3本を全車両が個別に保有。heavy benchで計100MB規模

## 3. 設計方針

### 3.1 ECSの採否

完全なECSフレームワーク（entt等の外部ライブラリ，アーキタイプ，動的コンポーネント合成）は**採用しない**。
理由: エンティティ種別はNode/Link/Vehicleの3種で固定，コンポーネント構成は実行時に変化しない，外部依存は本家UXsimに持ち込めない。
採用するのはECSの核となる考え方のみ: **整数ID（dense index）でエンティティを参照し，World が状態をコンポーネント別のSoA配列として保有し，メインループは状態別（HOME/WAIT/RUN）に分離されたコンテナを走査するシステム関数として書く**。
以下「SoAコア」と呼ぶ。

### 3.2 3層構造

```
[Python] uxsim_cpp_wrapper.py（CppWorld/CppNode/CppLink/CppVehicle） … 原則無変更
   │
[bindings] bindings.cpp（nanobind） … def_ro→def_prop_ro等の機械的変更のみ
   │
[C++ファサード] Vehicle/Node/Link 構造体 … idx＋coldデータ保持，hotデータはWorld配列へのアクセサ
   │
[SoAコア] World のコンポーネント配列群＋状態別コンテナ＋システム関数
```

- **SoAコア**: `World` に `veh_x[]`, `veh_x_next[]`, `veh_v[]`, `veh_move_remain[]`, `veh_state[]`, `veh_link[]`（link id），`veh_lane[]` 等のhot配列を追加。メインループのシステム関数はこれらを直接走査
- **C++ファサード**: `Vehicle` 構造体は `World* w; int idx;` とcoldデータ（name, departure_time, 経路指定, ログ等）を保持。hot変数はアクセサ経由で配列を読む。エンジン外から使う操作（DTAソルバー，bindings）はこのファサードを通る
- **bindings**: `def_ro("x", &Vehicle::x)` を `def_prop_ro("x", ...)` に置換する類の機械的変更。属性名・型・セマンティクス（コピー返却，def_rwの書き込み反映先）は完全維持
- **ラッパー**: 読み書きする属性名が不変なので原則無変更。`link._cpp_link.signal_group` 直接代入等の既存の裏口的アクセスパスも維持する

### 3.3 主要な設計変更点

#### (a) Vehicle hot stateのSoA化

hot配列: `x, x_next, x_old, v, move_remain`（double），`state`（int8可），`link`（int32 link id, -1=なし），`lane`（int32）。
2026-04-01の旧SoA計画（`20260401_plan_soa_vehicles.md`，未完遂）と同方向だが，今回はファサード込みで完遂させる。

#### (b) リンク車両キューのインデックス化とleader導出

`deque<Vehicle*>` → 車両idxのring buffer（`vector<int32>`＋head/tail）。
キュー操作はpush_back（generate/transfer流入）とpop_front（transfer流出/end_trip）のみなのでring bufferで完全に表現できる。
leaderは「リンクキュー内で `number_of_lanes` 個前の車両」なので，**leader/followerポインタの保持と繋ぎ替え管理を廃止し，キュー位置からの導出に置き換える**。
境界条件（transferでの `x_cong` 計算がleaderの `x_old` を使う等）はPython版と1行ずつ突き合わせて等価性を確認する。
導出化が難しい箇所が残る場合は，leader/followerをint32インデックス配列として残す縮退案を取る。

#### (c) メインループの状態別システム化（乱数順序制約の撤廃による本丸）

乱数消費順序を保つ必要がなくなったため，「全車両をid順に走査する」構造そのものを廃止し，状態別のコンテナを状態別のシステム関数で処理する形に再設計する。

- **HOME→WAIT（出発処理）**: 出発タイムステップをキーとするバケット（`vector<vector<int32>>` または事前ソート済み配列＋カーソル）を finalize 時に構築。毎stepは該当バケットの車両だけを generation_queue へ移す。HOME車両の毎step走査（Round 6計測で残存走査の57%）が完全に消える。シミュレーション途中の adddemand・`departure_time` 書き換え（def_rw）はチャンク境界でのバケット再構築で対応（P1のupdate_order再構築と同じ考え方）
- **WAIT**: 現行どおりノードの generation_queue（車両idxのdeque）で管理。追加走査は不要
- **RUN（移動・状態遷移・ログ）**: 車両の走査を**リンクごとのキュー順**に統一する。car_follow（x_next計算）と位置更新・リンク末端到達判定・log_dataを，リンク順の融合パス（1〜2パス）として書く。`x[queue[i]]` と `x[queue[i-lanes]]` の規則的アクセスになり，キャッシュ・SIMDに適する。`update_order` リストは廃止
- **END/ABORT**: どのコンテナにも属さないため走査コストゼロ（現行の圧縮処理も不要になる）

乱数を消費する箇所（generate/transferの合流抽選，route choiceの選択）はシステム化後も残るが，消費順序が変わるだけで各抽選の分布は不変であり，結果は統計的に同等となる。

#### (d) タイブレークの正準順序

hard_deterministic_mode のタイブレーク（merge_priority同点時は先頭優先，preference同点時は先頭優先）が走査順に依存して変わらないよう，候補列の順序を正準化する。
具体的には，ノードの `incoming_vehicles` は車両id順を正準順序とする（現行実装もid順走査の結果としてid順になっているため，これにより乱数非依存シナリオのビット同一性が旧実装に対して維持できる）。
incoming列は小さいので，リンク順パスで積んだ後にid順ソートしても正準挿入してもコストは無視できる。

#### (e) ログのアリーナ化（measure-first）

per-vehicleの5配列（link, x, v, lane, s）を，グローバルアリーナからチャンク（例: 512エントリ）を配布するチェーン方式，または現行方式のまま配置だけ改善する方式を比較する。
Round 4でAoSパック化がエクスポート時transpose増で+18%退行した履歴があるため，**プロトタイプでA/B計測してから本実装**する。
エクスポート（`build_all_vehicle_logs_flat_compact`）とチャンク実行途中の読み出しの両方で退行しないことが採用条件。

#### (f) per-vehicle固定費の削減

- `Vehicle::route_preference`: エンジン内部・ラッパーとも未使用（World側を使用，Round 6調査）。bindingsのdef_rwを保ったままlazy化
- `_traveled_nodes`: per-vehicle `vector<bool>` O(N) → グローバルbitmapアリーナまたは世代カウンタ方式
- `_buf_outlinks` 等の選択バッファ3本: World共有スクラッチへ

#### (g) Node/Linkは原則現状維持

個数が少なくホットパス比重も小さいため，オブジェクト構造は変えない。
変更はリンク車両キュー（b）と，車両参照のポインタ→idx置換のみ。
`route_preference[dest][link]` の flat 化（stride L の単一配列）は Phase 5 の計測次第。

#### (h) 乱数ストリームと並列化（2026-07-15改訂: ユーザー承認によりスコープに追加）

Phase 4完了時点の再評価（§8）で「1スレッド・レイアウト変更のみでは当初目標に到達不能」と結論し，ユーザー承認のもと決定的並列化をスコープに追加した。

- **Phase 7（並列化準備，直列のまま）**: 単一 `mt19937` をper-nodeストリーム（transferのシャッフル・合流抽選，generate時の経路選択）とper-linkストリーム（RUNパスでの経路選択，adj旅行時間ノイズ）に分離。シードとentity種別・idから決定的に初期化。併せて並列化を阻害する共有可変状態を排除（vehicles_living/runningの監査・削除，統計和のper-entity部分和＋id固定順リダクション，incoming_vehiclesのper-link収集バッファ化）
- **Phase 8（OpenMP並列化）**: links update（per-link），generate/signal/capacity/transfer（per-node。ノードのin/out linkは当該ノード専有なので排他），RUN融合パス（per-link），WAIT/HOMEパス，route_search_all（per-source），route_choice_duo（per-dest）を静的分割で並列化。entity別RNGと固定順リダクションにより**同一シードならスレッド数に依らずビット同一**を成立させる
- ベンチ規約: 1スレッド計測（対Python比・非退行確認）を維持し，4/8スレッドのスケーリング計測を併記する

### 3.4 変えないもの（互換性制約）

以下は挙動・値・型レベルで完全維持する。

1. **乱数非依存シナリオのビット同一性**: hard_deterministic_mode，および経路自由度・合流競合の抽選が結果に影響しないシナリオでは，リファクタ前後で全出力（全車両ログ，traveltime，累積曲線，TTT）がビット単位一致すること（(d)の正準順序により担保）
2. **確率的シナリオの統計的同等性**: 30シードの試行で主要指標（TTT，完了トリップ数，平均速度，リンク交通量）が統計的に同等であること（検定方法は§5）
3. **bindings API**: 全属性名・メソッド名・戻り値型・例外型。def_rwフィールドへのPython側からの書き込み（`state`, `move_remain`, `departure_time`, `signal_group` 等）がエンジンに反映されるセマンティクス
4. **逐次シナリオ構築**: addNode→addLink→adddemandの逐次登録，`set_t_max` による後からのリサイズ，チャンク実行（`duration_t`/`until_t`）と境界での読み出し・介入（adddemand追加，change_free_flow_speed等）
5. **DTAソルバー**（`dta_solver.cpp`）: Vehicleメンバ直接アクセス箇所はファサード経由に機械的に追従。ソルバーロジック自体（swap抽選等）は変更しない
6. **ラッパーの既存仕様**: `CppNode.signal` getterのコピー返却，`CppLink.signal_group` の`__init__`時のみ転送，等の既知の細部

## 4. フェーズ計画

各フェーズの完了条件（ゲート）:

- **G1（テスト）**: `test_cpp_mode.py` 全件通過
- **G2（決定的等価）**: 乱数非依存シナリオ集（§5.1）で対リファクタ前HEADビルドのビット同一
- **G3（統計的等価）**: 確率的シナリオで30シード統計的同等（§5.2）。**乱数消費順序を変えないフェーズでは，より強い「確率的シナリオ含む全ビット同一」で代替する**
- **G4（性能）**: ベンチ非退行。効果が誤差内で複雑さだけ増えるフェーズはrevert（Round 4の先例）

### Phase 0: 検証ハーネスとベースライン確定

- 乱数非依存シナリオ集の整備: 直列渋滞・abort・信号・ノード容量・多車線・チャンク実行＋介入を，hard_deterministic_mode および経路自由度なしネットワークで構成（乱数順序が変わっても結果が変わらないことが確認できる設計にする）
- 統計的同等性ハーネス: 30シード×新旧2エンジンで主要指標を収集し，§5.2の判定を自動化（`tmp/ecs_refactor/` に配置）
- ベンチスイート固定: 11x11 heavy（コアシム）＋log-heavy（エクスポート系），OMP_NUM_THREADS=1・10回・中央値±std・インターリーブA/B
- 現HEADでベースライン計測とプロファイル再取得（Round 6から日が経っているため）
- 成果物: ベースライン数値を本ファイルに追記

### Phase 1: Vehicle hot stateのSoA化＋ファサード

- `Vehicle` に `idx` 導入，hot変数8本をWorld配列へ移設，エンジン内の全参照を配列アクセスに書き換え
- bindings.cppのdef_ro/def_rwをdef_prop_ro/def_prop_rwに置換（値・型・書き込み反映は不変），dta_solver.cppの参照箇所を追従
- 走査順は変えないため**全ビット同一**（G3強化版）をゲートとする

### Phase 2: リンクキューのインデックス化とleader導出化

- `deque<Vehicle*>` → int32 ring buffer，leader/followerポインタ廃止（不可なら int32 配列化に縮退）
- generate/transfer/end_tripの繋ぎ替えロジックをキュー位置導出に置換
- 挙動等価性の最難所のため，Python版との1行対応表を作って検証する。走査順は変えないため**全ビット同一**をゲートとする

### Phase 3: メインループの状態別システム化

- HOME出発バケット導入，`update_order` 廃止，RUN処理（car_follow＋移動＋ログ）のリンク順融合パス化，incoming_vehiclesの正準id順化
- 乱数消費順序がここで初めて変わる。ゲートはG2（決定的ビット同一）＋G3（30シード統計的同等）
- 予想効果: 死に走査の完全除去＋規則的メモリアクセスで，本リファクタ最大の高速化を見込む

### Phase 4: ログアリーナ（measure-first）

- プロトタイプでチャンクチェーン方式 vs 現行方式をA/B計測し，シミュレーション側・エクスポート側の合計で勝つ場合のみ採用

### Phase 5: 固定費削減と計測ベースの追加最適化

- route_preference lazy化，_traveled_nodesアリーナ化，選択バッファ共有化
- 計測で効果があれば: World route_preferenceのflat化，car_followのSIMD明示化
- （検討のみ・デフォルト無効）car_followのOpenMP並列化

### Phase 7: 並列化準備（§3.3(h)。Phase 5の後に実施，直列のまま）

- RNGストリーム分離と共有状態排除。確率的結果の変化を並列化本体から分離するのが目的
- ゲート: det_suiteグループAビット同一＋stat_suite 30シード統計的同等＋1スレッドベンチ非退行。通過後にグループB新リファレンスを記録

### Phase 8: OpenMP決定的並列化（§3.3(h)）

- ゲート: OMP_NUM_THREADS=1/2/8でPhase 7とビット同一（det_suite全10シナリオ），1スレッドベンチ非退行，4/8スレッドのスケーリング計測

### Phase 6: 総合検証とPR準備（最終フェーズとして実施）

- CLAUDE.mdのPR前必須チェック: 全リグレッション，Python版との妥当性検証（30シード規模の統計比較，§5.2と同じ手法をPython vs 新C++に適用），精密ベンチマーク（スピードアップ倍率記載）
- 既存テストのうち固定シードの数値期待に依存するものの点検（§6リスク参照）
- devlogに完了報告，本家へのPR分割方針の決定（フェーズ単位で分割PRにするか一括か，サイズを見て判断）

## 5. 等価性の検証方法

### 5.1 乱数非依存シナリオ（ビット同一，G2）

Round 5/6で使ったsha256方式（全車両ログ・全リンクtraveltime・累積曲線・TTT）を新旧エンジンのビルド切替で比較する。
シナリオ集はPhase 0で整備し，合流・信号・容量・abort・多車線・チャンク介入の各メカニズムを乱数の影響なしで通す構成にする。

### 5.2 確率的シナリオ（統計的同等，G3）

`devlog/20260711_large_scenario_corr_investigation.md` で確立したwithin/cross比較の方法論を流用する。

- 30シード×新旧2エンジンで実行し，指標: TTT，完了トリップ数，平均速度，リンク別交通量・旅行時間
- スカラー指標: Welch t検定でp>0.05（帰無仮説: 平均が等しい）を確認。加えて平均差が数%以内であること
- リンク別指標: cross相関（新vs旧）がwithin相関（旧vs旧の別シード）と同水準であることを確認（系統差があればcross < withinとなる）
- シナリオは11x11 heavy＋信号8x8の2系統

## 6. 期待効果とリスク

### 期待効果（正直な見積り）

- main_loop（heavy bench 約1.5s）: 死に走査の完全除去・リンク順の規則的アクセス・ログ書き込み改善で**1.5〜2.5x**を狙う（乱数順序制約の撤廃により当初計画の1.5〜2.0xから上方修正）
- exec_simulation全体: Pythonラッパー後処理約1.3sは本リファクタの対象外のため，全体では**1.3〜1.6x**程度に希釈される見込み
- vehicle_log_mode=0 やDTAソルバー反復のようなmain_loop比重が高い用途ではより大きく効く
- メモリ: per-vehicle固定費削減で数百MB規模の削減（heavy bench）

### リスク

| リスク | 対策 |
|---|---|
| 統計的同等ゲートは微小なバグを見逃しうる | 乱数非依存シナリオ集（G2）を各メカニズム網羅で厚く作り，ロジック等価性はビット同一で担保。統計ゲートは乱数順序変更の影響確認に限定して使う |
| 既存テストの固定シード数値期待が乱数順序変更で外れる | 従来方針「乱数列の差分はC++側で無理に合わせず，テスト側でrel_tol」に従い点検・調整。調整したテストは完了報告に列挙する |
| leader導出化の境界条件差（x_old参照，abort，信号待ち） | Python版との1行対応表を先に作る。導出化不能ならインデックス配列に縮退 |
| ログアリーナがエクスポートで退行（Round 4再来） | プロトタイプA/B計測後にのみ本実装 |
| hard_deterministicのタイブレークが走査順変更で変わる | incoming_vehicles等の候補列をid順に正準化（§3.3(d)） |
| bindingsのプロパティ化でPython側アクセスが遅くなる | def_prop化の前後でラッパー経由アクセスのマイクロベンチ。ラッパーはnumpy一括APIを使っており個別アクセスはホットパスでないが計測で裏取り |
| DTAソルバーの追従漏れ | `test_cpp_mode.py` 内の既存DTAテストで担保（専用検証はスコープ外） |
| 旧SoA計画（2026-04-01）が未完遂に終わった前例 | 今回はファサード方式で影響範囲を限定し，フェーズごとにマージ可能な状態を保つ |
| 本家PRとしてのレビュー負荷 | フェーズ単位の独立コミット。PR分割はPhase 6で判断 |

## 7. 作業運用

- 開発は本ブランチ `refactor/ecs-soa-engine` で行い，`git push fork refactor/ecs-soa-engine` で随時push
- 期間中に本家mainが進んだ場合は都度rebaseして追従（特にuxsim.py側の新機能追従タスクと衝突しないよう，追従タスクを優先）
- ベンチ実行中は他プロセスを走らせない
- 各フェーズ完了時にdevlogへ計測結果を追記

## 8. 経過記録

### Phase 0 完了（2026-07-15）

- ハーネス3本を `tmp/ecs_refactor/` に整備（det_suite: 10シナリオsha256，stat_suite: 30シードWelch+within/cross相関，bench_suite: 1スレッド10回中央値）
- ベースライン（HEAD 08df4fb）: test_cpp_mode.py 211件通過。heavy_grid exec 2.093s±0.251，log_heavy logbuild 1.290s±0.035。det自己一致10/10
- 注意: ベンチはプロセス起動間で約15%ドリフトするため，フェーズ判定はインターリーブA/B（交互ビルド）で行う

### Phase 1 完了（2026-07-15, コミット 68da00c）

- Vehicle hot 8変数をWorld SoA配列へ移設，Vehicleは idx＋インラインアクセサのファサードに。bindingsはdef_prop化，wrapper無変更
- ゲート: det_suite 10/10ビット同一（確率的シナリオ含む），test_cpp_mode.py 211件通過
- **ベンチ退行 +16%**（インターリーブ3ラウンド: base中央値2.14〜2.45s vs Phase 1 2.65〜2.69s）。原因はアクセサ経由の間接参照（Vehicle構造体ロード＋vector基底ロードの増加）で，Phase 3のホットループ配列直接走査化で解消される構造のもの
- **判断**: Phase 1は土台フェーズとして退行を暫定受け入れ。**回復条件: Phase 3完了時点でheavy_grid execがベースライン（約2.1〜2.3s）を明確に下回ること**。満たさない場合はSoA方針自体を再評価する

### Phase 2 完了（2026-07-15, コミット 1903ed7）

- リンク車両キューをint32リングバッファ＋単調シーケンス番号に置換，leader/followerポインタと繋ぎ替えコードを全廃（位置導出方式）。Python版・旧C++との9項目対応表で等価性確認
- ゲート: det_suite 10/10ビット同一，test_cpp_mode.py 211件通過，dta_solver.cpp変更不要
- ベンチ: Phase 1比さらに約+15%（`leader()`導出の依存ロード連鎖）。土台フェーズとして受け入れ（累積 対base約+33%）

### Phase 3 完了（2026-07-15, コミット 902500f）

- update_order・Vehicle::update()・car_follow_newell()を廃止し，タイムステップ内を links → generate/signal/capacity → transfer → **RUN融合パス（リンク順）** → WAITログパス → HOME出発バケットパス → route choice に再構成。incoming_vehiclesはtransfer冒頭でid昇順に正準化
- **仕様修正（重要な発見）**: 計画時の想定「log_sのleader位置はx_old読み替えで等価」は誤り。旧コードのlog_sはid順走査に依存し，leaderのidが自車より小さければ**移動後のx**，大きければ移動前のxを読んでいた。実装は `leader_id < self_id ? x : x_old`（およびleaderが同stepで終了済みの場合は-1）の判別則で旧挙動をビット単位再現した（担当エージェントが実測で検証・特定）
- ゲート: det_suiteグループA 7/7ビット同一（merge_hard_det, grid_hard_det含む），グループBは想定通りFAIL（乱数消費順序変更）。test_cpp_mode.py 211件通過（テスト調整は不要だった）。stat_suite全PASS（heavy: TTT差−1.05% p=0.431, cross相関0.417≧within 0.410 / signal: TTT差+0.07% p=0.944, cross 0.278≧within 0.270）
- ベンチ（3-legインターリーブ, base/phase2/phase3）: phase3はphase2の退行を全額回収（各ラウンド30〜40%高速）。対baseはラウンド中央値で 2.135/2.351/2.583 vs 2.291/2.341/2.346 とパリティ〜微改善（5ラウンド追試で中央値の平均−9%，記録ベースライン比−13.7%）。**回復条件「baseを明確に下回る」は厳密には未達**（マシンノイズ±15%が信号5〜9%を上回る）
- プロファイル: main_loop 1.448sのうちRUN融合パス1.220s（84%），支配項は**per-vehicleログ5配列へのpush_back**（baseも同コストを払っている共通ボトルネック）。構造改革によるアクセサ退行の解消は完了し，残る高速化レバーはPhase 4のログアリーナ
- **判断**: 対base非退行・構造目標達成・ボトルネックがPhase 4スコープと特定できたため受け入れ。**明確なベンチ勝利の実現はPhase 4に委ねる**（Phase 4で達成できなければ§5の回復条件不達としてプロジェクト全体を再評価）

### Phase 4 完了・不採用（2026-07-15, コミットなし・revert済み）

- timestep-majorグローバルフレームログ＋遅延gather（カウンティングソート，エポック無効化）を実装。正確性は全通過（det_suite 10/10ビット同一 vs Phase 3リファレンス，テスト211件）だったが，**採用基準(i)不成立: heavy execがPhase 3比パリティ〜悪化**（3ラウンド: 2.332/2.072/2.144 vs 2.331/2.382/2.340。リザーブ理想化プローブでも+6.5%）のためrevert
- **教訓**: ログコストの本質は書き込み量であり書き込み先の散在ではない。per-vehicle配列はreserve済みで各車両の処理中はキャッシュに乗っており，シーケンシャル化はframe_veh列の追加書き込みとエクスポート時gatherで相殺される（Round 4の教訓の再確認）
- 付随した確認: ピークRSSはper-vehicle reserveが遅延コミットのため両者同等（約6.0GB）。Phase 4以降のビット同一リファレンスは `tmp/ecs_refactor/baseline_det_phase3.json`

### 再評価（Phase 4完了時点，§5回復条件に基づく）

- 現状の対base性能: heavy execで−5〜−13%（インターリーブで−5〜−9%，記録ベースライン比−13.7%）。マシンノイズ±15%に対しパリティ〜小幅改善であり，**当初目標のmain_loop 1.5〜2.5xは1スレッド・レイアウト変更のみでは到達不能**と結論
- 理由: (i) 死に走査の除去はRound 6のP1（update_order圧縮）で既にbaseに取り込まれており，本リファクタの上積みはHOMEバケット等に限られた，(ii) 残る支配項（per-vehicleログ書き込み）は書き込み量が本質でbaseも同額を払う共通コスト
- 得られたもの: SoAコア＋状態別システム＋位置導出リンクキューという並列化・SIMD適性の高い構造，全互換（wrapper無変更・テスト無調整），統計的同等性の実証，メモリ削減余地（Phase 5）
- **残る大きな高速化レバーは並列化**（乱数消費順序制約の撤廃とリンク独立なRUNパス構造により，per-linkストリームRNG＋静的分割で決定的並列が設計可能になった）。ただしベンチ規約は1スレッドでありスコープ拡張はユーザー判断待ち。Phase 5（固定費削減）は計画どおり実施する

### 方向性決定（2026-07-15，ユーザー承認）

再評価を受けてユーザーに方向性を確認し，「並列化に進む」が選択された。
Phase 5 → Phase 7（並列化準備）→ Phase 8（OpenMP決定的並列化）→ Phase 6（総合検証・PR準備）の順で進める。

### Phase 5 完了（2026-07-15, コミット 29e67b1）

- 採用3項目: (1) Vehicle::route_preference lazy化（エンジン未使用をgrepで再確認，bindingsはdef_prop_rwで観測可能挙動維持。RSS −75MB），(2) ログreserveの出発時右サイズ化（上界 total_timesteps − timestep + 1 を導出，realloc 0回を計測で確認。VmPeak −417MB），(3) 経路選択バッファのWorld共有化（並列化時はthread_local化する旨をヘッダに注記）
- スキップ2項目: _traveled_nodes（約320KBで計測フロア未満），SIMD car_follow（gatherアクセスでROI不足）
- ゲート: det_suite 10/10ビット同一，テスト211件通過。heavy build 0.125→0.078s，total 中央値2.32→1.90s，exec非退行（1.81〜1.83sで安定），log_heavy改善（logbuild 1.43→1.28s）

## 9. 参考資料

- プロファイル・P1/P2実装: `devlog/20260703_plan_cpp_core_optimization_round6.md`
- AoSログパック放棄の経緯: `devlog/20260401_plan_cpp_optimization.md`（Round 4）
- 旧SoA計画: `devlog/20260401_plan_soa_vehicles.md`
- ビット同一性検証・ベンチ手法: `devlog/20260702_round5_final.md`
- 統計的同等性の判定方法論（within/cross比較）: `devlog/20260711_large_scenario_corr_investigation.md`
- Python等価性の検証実験: `tmp/precise_compare/experiments.py`（現存），`devlog/20260711_cpp_python_precise_comparison_and_fixes.md`
