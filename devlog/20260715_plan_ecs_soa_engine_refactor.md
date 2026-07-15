# 計画: C++エンジン内部のECS/SoA化リファクタ

作成: 2026-07-15
ブランチ: `refactor/ecs-soa-engine`
ステータス: 計画（未着手）

## 1. 目的

C++エンジン（`uxsim/trafficpp/traffi.h`, `traffi.cpp`）の内部データ構造をデータ指向（SoA: Structure of Arrays）に抜本的に再設計し，シミュレーション本体を高速化する。
外部から見える機能・インターフェース（`World(cpp=True)` のPython互換API，bindings，ラッパー，analyzer連携，DTAソルバー）は一切変えない。
これを実現するため，SoAコアの上に既存のアクセス方法を提供するC++ファサード層を設け，bindings.cpp・uxsim_cpp_wrapper.py の変更を最小限に抑える。

## 2. 現状アーキテクチャの分析

### 2.1 データ構造（AoS）

- `World` がエンティティレジストリ（`vector<Vehicle*>`, `vector<Link*>`, `vector<Node*>`）とグローバル配列（`route_preference`, `adj_mat_time`, `rsa_*` スクラッチ）を保持
- `Node` / `Link` は少数（典型的に数十〜数百個）のヒープオブジェクト。リンクの時系列（`arrival_curve` 等）は既に連続配列であり，キャッシュ効率の問題は小さい
- `Vehicle` は多数（数万〜数十万プラトーン）の個別ヒープオブジェクト。ホット変数（`x`, `x_next`, `v`, `move_remain`, `state`, `link`, `leader`）とコールドデータ（`name`, ログ5配列，`route_preference` O(L)，`_traveled_nodes` O(N)，バッファ3本）が混在し，1台あたり数百バイト＋可変長ベクトル群
- リンク上の車両キューは `deque<Vehicle*>`。leader/follower はポインタで相互リンクし，generate/transfer/end_trip の3箇所で繋ぎ替え管理

### 2.2 プロファイル（Round 6, 2026-07-03, 11x11グリッド heavy bench）

P1（update_order）+P2（log_t/log_stateスカラー化）適用後の構成が現在のHEAD。

| 項目 | 時間 | 備考 |
|---|---|---|
| exec_simulation全体 | 2.81s | |
| ├ C++ main_loop | 約1.5s | |
| │ ├ veh_update | 約1.0s | 大半がlog_data（5配列へのpush_back） |
| │ ├ car_follow | 約0.3s | leader->x のランダムアクセス |
| │ ├ node_transfer | 約0.14s | |
| │ └ link_update他 | 約0.1s | |
| └ Pythonラッパー後処理 | 約1.3s | `_build_all_vehicle_log_caches` 等 |

- 対Pythonのスピードアップ実績: 15.2x（9x9）〜31.5x（信号8x8）
- ピークRSS約2.2GB（ログ配列とper-vehicle固定費が支配的）

### 2.3 AoS構造に起因する非効率（本リファクタの対象）

1. **log_dataのライトストリーム分散**: 毎step毎車両，車両ごとに別ヒープ領域にある5本のvectorへpush_back。書き込み先がRUN車両数ぶん散在する
2. **car_followのポインタ間接**: `update_order`（id順）で走査するため，`veh->x` も `leader->x` もヒープ上に散在。リンク内の車列は本来規則的（leaderはリンクキューの `number_of_lanes` 個前）なのに，その構造を活かせていない
3. **per-vehicle固定費**: `route_preference`（O(L)，エンジン内部では未使用），`_traveled_nodes`（O(N)），選択バッファ3本を全車両が個別に保有。heavy benchで計100MB規模
4. **Vehicleオブジェクトのサイズと配置**: state判定だけのためにも車両全体のキャッシュラインを引き込む

## 3. 設計方針

### 3.1 ECSの採否

完全なECSフレームワーク（entt等の外部ライブラリ，アーキタイプ，動的コンポーネント合成）は**採用しない**。
理由: エンティティ種別はNode/Link/Vehicleの3種で固定，コンポーネント構成は実行時に変化しない，外部依存は本家UXsimに持ち込めない。
採用するのはECSの核となる考え方のみ: **整数ID（dense index）でエンティティを参照し，World が状態をコンポーネント別のSoA配列として保有し，メインループは配列を直接走査するシステム関数として書く**。
以下「SoAコア」と呼ぶ。

### 3.2 3層構造

```
[Python] uxsim_cpp_wrapper.py（CppWorld/CppNode/CppLink/CppVehicle） … 原則無変更
   │
[bindings] bindings.cpp（nanobind） … def_ro→def_prop_ro等の機械的変更のみ
   │
[C++ファサード] Vehicle/Node/Link 構造体 … idx＋coldデータ保持，hotデータはWorld配列へのアクセサ
   │
[SoAコア] World のコンポーネント配列群＋システム関数（car_follow_system等）
```

- **SoAコア**: `World` に `veh_x[]`, `veh_x_next[]`, `veh_v[]`, `veh_move_remain[]`, `veh_state[]`, `veh_link[]`（link id），`veh_lane[]` 等のhot配列を追加。メインループのシステム関数はこれらを直接走査
- **C++ファサード**: `Vehicle` 構造体は `World* w; int idx;` とcoldデータ（name, departure_time, 経路指定, ログ等）を保持。hot変数はアクセサ経由で配列を読む。エンジン外から使う操作（DTAソルバー，bindings）はこのファサードを通る
- **bindings**: `def_ro("x", &Vehicle::x)` を `def_prop_ro("x", ...)` に置換する類の機械的変更。属性名・型・セマンティクス（コピー返却，def_rwの書き込み反映先）は完全維持
- **ラッパー**: 読み書きする属性名が不変なので原則無変更。`link._cpp_link.signal_group` 直接代入等の既存の裏口的アクセスパスも維持する

### 3.3 主要な設計変更点

#### (a) Vehicle hot stateのSoA化

hot配列: `x, x_next, x_old, v, move_remain`（double），`state`（int8可），`link`（int32 link id, -1=なし），`lane`（int32）。
`update_order` は `vector<int32>`（車両idx）になる。
2026-04-01の旧SoA計画（`20260401_plan_soa_vehicles.md`，未完遂）と同方向だが，今回はファサード込みで完遂させる。

#### (b) リンク車両キューのインデックス化とleader導出

`deque<Vehicle*>` → 車両idxのring buffer（`vector<int32>`＋head/tail）。
キュー操作はpush_back（generate/transfer流入）とpop_front（transfer流出/end_trip）のみなのでring bufferで完全に表現できる。
leaderは「リンクキュー内で `number_of_lanes` 個前の車両」なので，**leader/followerポインタの保持と繋ぎ替え管理を廃止し，キュー位置からの導出に置き換えられる**。
ただしログ（`log_s`）と `car_follow` が参照するのは「同一リンク上のleader」であり，境界条件（transferでの `x_cong` 計算はleaderの `x_old` を使う等）をPythonと1行ずつ突き合わせて等価性を確認する。
導出化が難しい箇所が残る場合は，leader/followerをint32インデックス配列として残す（ポインタ→インデックスの置換だけでも効果はある）。

#### (c) car_followのリンク順走査化

現行はid順（update_order）で `car_follow_newell` を呼ぶが，この関数はRNGを消費せず書き込みは自車の `x_next`/`move_remain` のみで順序非依存（読むのは前ステップ確定値の `leader->x`）。
よって**リンクごとにキュー順で走査する形に並べ替えてもビット同一**。
リンク順走査では `x[queue[i]]` と `x[queue[i - lanes]]` という規則的アクセスになり，キャッシュ・SIMD・（将来の）並列化に適する。
一方 `Vehicle::update()`（状態遷移・log・incoming_vehicles登録）は `incoming_vehicles` への登録順がtransferのRNG消費順に影響するため**id順を厳守**する。

#### (d) ログのアリーナ化（measure-first）

per-vehicleの5配列（link, x, v, lane, s）を，グローバルアリーナからチャンク（例: 512エントリ）を配布するチェーン方式，または現行方式のまま配置だけ改善する方式を比較する。
Round 4でAoSパック化がエクスポート時transpose増で+18%退行した履歴があるため，**プロトタイプでA/B計測してから本実装**する。
エクスポート（`build_all_vehicle_logs_flat_compact`）とチャンク実行途中の読み出しの両方で退行しないことが採用条件。

#### (e) per-vehicle固定費の削減

- `Vehicle::route_preference`: エンジン内部・ラッパーとも未使用（World側を使用，Round 6調査）。bindingsのdef_rwを保ったままlazy化
- `_traveled_nodes`: per-vehicle `vector<bool>` O(N) → グローバルbitmapアリーナまたは世代カウンタ方式
- `_buf_outlinks` 等の選択バッファ3本: World共有スクラッチへ

#### (f) Node/Linkは原則現状維持

個数が少なくホットパス比重も小さいため，オブジェクト構造は変えない。
変更はリンク車両キュー（b）と，車両参照のポインタ→idx置換のみ。
`route_preference[dest][link]` の flat 化（stride L の単一配列）は Phase 5 の計測次第。

### 3.4 変えないもの（互換性制約）

以下は挙動・値・型レベルで完全維持する。

1. **数値的ビット同一性**: リファクタ前後のC++エンジンで，決定的・確率的シナリオともに全出力（全車両ログ，traveltime，累積曲線，TTT）がビット単位一致すること。RNG消費順序と浮動小数点演算順序を保存する設計とし，並べ替えるのは順序非依存を確認した箇所（car_follow）のみ
2. **bindings API**: 全属性名・メソッド名・戻り値型・例外型。def_rwフィールドへのPython側からの書き込み（`state`, `move_remain`, `signal_group` 等）がエンジンに反映されるセマンティクス
3. **逐次シナリオ構築**: addNode→addLink→adddemandの逐次登録，`set_t_max` による後からのリサイズ，チャンク実行（`duration_t`/`until_t`）と境界での読み出し・介入（adddemand追加，change_free_flow_speed等）
4. **DTAソルバー**（`dta_solver.cpp`）: Vehicleメンバ直接アクセス箇所はファサード経由に機械的に追従させ，結果はビット同一
5. **ラッパーの既存仕様**: `CppNode.signal` getterのコピー返却，`CppLink.signal_group` の`__init__`時のみ転送，等の既知の細部

## 4. フェーズ計画

各フェーズの完了条件（ゲート）は共通で，(i) `test_cpp_mode.py` 全件通過，(ii) ビット同一性検証（決定的1＋確率的2シナリオ，チャンク実行＋介入込み，対リファクタ前HEADビルド），(iii) ベンチ非退行（退行時はrevert）。
フェーズ順は依存関係順であり，各フェーズは独立にコミットする。

### Phase 0: 検証ハーネスとベースライン確定

- ビット同一性チェックスクリプト（Round 5/6のsha256方式を汎用化，`tmp/ecs_refactor/` に配置）
- ベンチスイート固定: 11x11 heavy（コアシム）＋9x9 DUE/DSO（ソルバー経由）＋log-heavy（エクスポート系），OMP_NUM_THREADS=1・10回・中央値±std・インターリーブA/B
- 現HEADでベースライン計測とプロファイル再取得（Round 6から日が経っているため）
- 成果物: ベースライン数値を本ファイルに追記

### Phase 1: Vehicle hot stateのSoA化＋ファサード

- `Vehicle` に `idx` 導入，hot変数8本をWorld配列へ移設，エンジン内の全参照を配列アクセスに書き換え
- bindings.cppのdef_ro/def_rwをdef_prop_ro/def_prop_rwに置換（値・型・書き込み反映は不変）
- dta_solver.cppの参照箇所を追従
- 予想効果: veh_updateループとstate判定のキャッシュ局所性改善（単体では小〜中，後続フェーズの土台）

### Phase 2: リンクキューのインデックス化とleader導出化

- `deque<Vehicle*>` → int32 ring buffer，leader/followerポインタ廃止（不可なら int32 配列化に縮退）
- generate/transfer/end_tripの繋ぎ替えロジックをキュー位置導出に置換
- ここが挙動等価性の最難所。Pythonコードとの1行対応表を作って検証する

### Phase 3: car_followのリンク順走査化

- リンクごとにring bufferを順走査するstencil型ループに変更（順序非依存性の根拠をコメントで明記）
- 予想効果: car_follow 0.3sの大幅短縮＋veh_update側のキャッシュ改善

### Phase 4: ログアリーナ（measure-first）

- プロトタイプでチャンクチェーン方式 vs 現行方式をA/B計測し，シミュレーション側・エクスポート側の合計で勝つ場合のみ採用
- Round 4放棄履歴（AoSパック+18%退行）があるため，採用判断は計測結果のみに基づく

### Phase 5: 固定費削減と計測ベースの追加最適化

- route_preference lazy化，_traveled_nodesアリーナ化，選択バッファ共有化
- 計測で効果があれば: World route_preferenceのflat化，car_followのSIMD明示化
- （検討のみ・デフォルト無効）car_followのOpenMP並列化。ビット同一を保てる場合のみ

### Phase 6: 総合検証とPR準備

- CLAUDE.mdのPR前必須チェック: 全リグレッション，Python版との妥当性検証（十分な規模・反復），精密ベンチマーク（スピードアップ倍率記載）
- 全demos_and_examplesのC++モード実行確認（既存テストでカバー）
- devlogに完了報告，本家へのPR分割方針の決定（フェーズ単位で分割PRにするか一括か，サイズを見て判断）

## 5. 期待効果（正直な見積り）

- main_loop（heavy bench 約1.5s）: ログ書き込み・car_follow・ポインタ間接の改善で**1.5〜2.0x**を狙う
- exec_simulation全体: Pythonラッパー後処理約1.3sは本リファクタの対象外のため，全体では**1.2〜1.4x**程度に希釈される見込み
- vehicle_log_mode=0 やDTAソルバー反復のようなmain_loop比重が高い用途ではより大きく効く
- メモリ: per-vehicle固定費削減で数百MB規模の削減（heavy bench）
- 効果が各フェーズのゲートで誤差内にとどまる場合，そのフェーズはrevertし複雑さを持ち込まない（Round 4の先例に従う）

## 6. リスクと対策

| リスク | 対策 |
|---|---|
| ビット同一性が崩れる（FP加算順・RNG順） | 順序を変えるのはcar_followのみ。フェーズごとにsha256検証。崩れたら原因特定まで先に進まない |
| leader導出化の境界条件差（x_old参照，abort，信号待ち） | Python版との1行対応表を先に作る。導出化不能ならインデックス配列に縮退 |
| ログアリーナがエクスポートで退行（Round 4再来） | プロトタイプA/B計測後にのみ本実装 |
| bindingsのプロパティ化でPython側アクセスが遅くなる | def_prop化の前後でラッパー経由アクセスのマイクロベンチ。ラッパーはnumpy一括APIを使っており個別アクセスはホットパスでないことを確認済みだが計測で裏取り |
| DTAソルバーの追従漏れ | dta_equivalenceテストとexample28で検証 |
| 旧SoA計画（2026-04-01）が未完遂に終わった前例 | 当時と違い今回はファサード方式で影響範囲を限定し，フェーズごとにマージ可能な状態を保つ |
| 本家PRとしてのレビュー負荷 | フェーズ単位の独立コミット。PR分割はPhase 6で判断 |

## 7. 作業運用

- 開発は本ブランチ `refactor/ecs-soa-engine` で行い，`git push fork refactor/ecs-soa-engine` で随時push
- 期間中に本家mainが進んだ場合は都度rebaseして追従（特にuxsim.py側の新機能追従タスクと衝突しないよう，追従タスクを優先）
- ベンチ実行中は他プロセスを走らせない
- 各フェーズ完了時にdevlogへ計測結果を追記

## 8. 参考資料

- プロファイル・P1/P2実装: `devlog/20260703_plan_cpp_core_optimization_round6.md`
- AoSログパック放棄の経緯: `devlog/20260401_plan_cpp_optimization.md`（Round 4）
- 旧SoA計画: `devlog/20260401_plan_soa_vehicles.md`
- ビット同一性検証・ベンチ手法: `devlog/20260702_round5_final.md`
- Python等価性の検証実験: `tmp/precise_compare/experiments.py`（現存），`devlog/20260711_cpp_python_precise_comparison_and_fixes.md`
