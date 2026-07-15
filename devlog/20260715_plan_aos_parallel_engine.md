# 計画: 非SoA（AoS維持）でのC++エンジン決定的並列化

作成: 2026-07-15
ブランチ: `refactor/aos-parallel-engine`（fork mainから分岐）
参考: `devlog/20260715_plan_ecs_soa_engine_refactor.md`（SoA版計画・§8経過記録），`devlog/20260715_ecs_soa_refactor_completion_report.md`

## 1. 背景と目的

ブランチ `refactor/ecs-soa-engine`（PR #340）ではECS/SoA全面リファクタの上に決定的OpenMP並列化を実装した。
しかしSoA化自体の1スレッド高速化効果は小さく（heavy execで−13.5%），高速化の主因は並列化（main_loop 8スレッドで2.76〜2.88x）だった。
本プロジェクトでは，**SoA化を一切行わず**（Vehicle/Link/Nodeのクラス構造・メンバ変数・deque・leader/followerポインタを維持），fork mainのAoSコードに同等の決定的並列化のみを実装する。

## 2. スコープと制約

- 出発点: fork main（origin/main 5ea3802 + フォーク専用コミット）
- **禁止事項: SoA化**。Worldへのコンポーネント配列移設，リングバッファ化，Vehicleのファサード化はしない
- 許可: 並列化に必要なメインループのステージ分割（走査順の再構成），RNGストリーム分離，共有可変状態の排除，thread_localスクラッチ
- 等価性要件（SoA版と同一）: 乱数非依存シナリオはビット同一，確率的シナリオは30シード統計的同等，スレッド数非依存ビット同一
- DUE/DSOはスコープ外（コアシミュレータに注力）

## 3. 設計方針（mainコードの分析に基づく）

mainのmain_loopは既に以下のステージ構造を持つ:

1. Link::update（per-link）
2. Node::generate + signal_update + flow_capacity_update（per-node）
3. Node::transfer（直列。共有outlinkキュー経由の逐次可視性セマンティクス）
4. car_follow_newellパス（update_order id順。**x_nextを計算するだけで適用は後段** → 順序非依存）
5. Vehicle::updateパス（id順。x適用・ログ・トリップ終了・次リンク選択）
6. 経路更新（route_search_all per-source，route_choice_duo per-dest）

順序依存があるのは (3) transfer と (5) のうち log_data の先行車位置参照（id順走査依存: leader_id < self_id なら移動後x）・end_tripカスケード（front-to-backなら保存）・HOME出発のgeneration_queue積み順（id順）のみ。

### 並列化設計（SoA版Phase 7/8/9の移植）

- **RNGストリーム分離**（SoA版 e80b91d 参照）: `World::rng` を per-node / per-link ストリームに分離（seed_seq{seed, kind, id}）。route_next_link_choiceはRNG参照を引数で受ける
- **共有可変状態の排除**: vehicles_living/running（読み出しなしなら削除），trips_completed_count等の統計和はper-entity部分和＋id昇順リダクション，incoming_vehiclesはper-link収集バッファ化しtransfer冒頭でid昇順集約
- **ステージ分割**（AoSのまま）: Vehicle::updateパスを RUN（per-link，リンクのvehicles dequeをfront-to-back走査）/ WAIT（per-vehicle，ログのみ）/ HOME（直列またはper-node）に分割。log_sは判別則 `leader_id < self_id ? 移動後x : 移動前x`（同stepで終了済みなら-1）でid順走査の挙動をビット単位再現
- **OpenMP並列化**（SoA版 1ecb30f 参照）: 1タイムステップを単一 `omp parallel` リージョン＋ステージ別 `omp for`（暗黙バリア）。transferは `omp single`。例外はリージョン内捕捉→最小entity id優先で決定的再送出。Dijkstra等のスクラッチはthread_local化。OpenMPフラグは**リポジトリ直下のCMakeLists.txt**に追加（scikit-build-coreはこちらを使う）
- **threads引数**（SoA版 4d2be00 参照）: `World(cpp=True, threads=1)`，デフォルト1，-1で全コア。全parallelリージョンに `num_threads(resolve_num_threads())`，omp_set_num_threadsは不使用

## 4. フェーズ計画

各フェーズをOpusエージェントに委任し，コーディネータがゲート判定する（SoA版と同じフェーズゲート方式）。
検証ハーネスは `tmp/ecs_refactor/`（git管理外）を再利用する。

### Phase 0: ハーネス適応とベースライン確認

- det_suite/stat_suite/bench_suite/scale_benchをthreads引数なしエンジンでも動くよう適応（threads対応は自動検出）
- 現mainコードが `baseline_det.json` と全10シナリオでビット同一であることを確認（ベースラインの有効性検証）
- ベンチベースラインを再計測して記録（マシンドリフト対策。以後の判定はインターリーブA/B）
- ゲート: det 10/10一致，ベンチ記録完了

### Phase 1: RNGストリーム分離と共有状態排除（直列のまま）

- SoA版Phase 7の移植。per-node/per-linkストリーム，統計の部分和化，incoming per-link収集バッファ化，vehicles_living/running監査
- ゲート: det_suiteグループA（乱数非依存7種）ビット同一，stat_suite 30シード統計的同等，test_cpp_mode.py 211件通過，ベンチ非退行（インターリーブ）。通過後にグループB新リファレンス記録

### Phase 2: メインループのステージ分割（直列のまま，AoS維持）

- Vehicle::updateパスの RUN per-link / WAIT / HOME 分割，log_s判別則，end_tripのリンク内カスケード保存
- ゲート: det_suiteグループAビット同一，stat_suite統計的同等，テスト211件，ベンチ非退行。通過後に全10シナリオの新リファレンス記録

### Phase 3: OpenMP決定的並列化

- SoA版Phase 8の移植。単一parallelリージョン＋ステージ別for，transferはsingle，thread_localスクラッチ，例外の決定的再送出，ルートCMakeLists.txtにOpenMP
- ゲート: threads 1/2/8で全10シナリオがPhase 2リファレンスとビット同一，テスト211件，1スレッドベンチ非退行，4/8スレッドスケーリング計測

### Phase 4: threads引数

- SoA版Phase 9の移植。bindings・wrapper（cppモード専用拡張）・バリデーション・回帰テスト1件追加
- ゲート: threads引数経由のスレッド数非依存ビット同一，テスト212件通過

### Phase 5: 総合検証と完了報告

- CLAUDE.mdのPR前必須チェック: 全リグレッション，python_parity（Python 10シード vs C++ 30シード），精密ベンチ（1スレッド複数シード中央値+std，スピードアップ倍率），スレッドスケーリング
- devlogに完了報告

## 5. 期待効果とリスク

- 期待: main_loopのスレッドスケーリングはSoA版と同水準（8スレッドで2.5〜2.9x）を狙う。1スレッド性能はmain比パリティ（SoA由来の−13.5%は放棄）
- リスク: AoSのポインタ追跡はキャッシュ局所性でSoAに劣り，スケーリングが低下する可能性 → Phase 3で計測し，SoA版との差を完了報告に記録
- リスク: dequeのfront-to-back走査とend_trip（pop_front）の同時進行 → イテレータ無効化に注意した実装（インデックス走査等）
- リスク: HOME出発・WAIT・print_progressの取りこぼし → Python版セマンティクスとの1行対応を各フェーズで実測検証

## 6. 経過記録

### Phase 0 完了（2026-07-15）

- ハーネス適応: `tmp/ecs_refactor/scenarios.py` の `_make_world()` にthreads引数サポートの自動検出（使い捨てCppWorldによるプローブ方式，キャッシュ付き）を追加．det/stat/benchの3ハーネスはここ経由のため1箇所で対応．scale_bench.pyは独自に同検出を追加（未対応時はOMP_NUM_THREADS方式＋警告）
- ビット同一性: 現AoSエンジン（5e8dd59）は `baseline_det.json`（リファクタ前 08df4fb）と**10/10ビット同一**．ベースラインの有効性を確認
- リグレッション: test_cpp_mode.py 211件通過（flaky 2件はrerunで通過）
- ベンチベースライン（`baseline_bench_aos_p0.json`, 1スレッド, n=10）: heavy_grid exec 1.777s±0.284，total 1.863s±0.350，log_heavy sim 0.143s±0.010，logbuild 1.142s±0.026
- 申し送り: heavy_gridのstdが約16%とマシンドリフト並みに大きい．フェーズ判定はインターリーブA/Bで行う（計画通り）．スケーリング判定はscale_benchのmainloop指標を使う

### Phase 1 完了（2026-07-15, コミット beddd7e）

- RNGストリーム分離: `World::rng` 廃止，`node_rngs`/`link_rngs`（`make_entity_rng(seed, kind, id)` = seed_seq{seed_lo, seed_hi, kind, id}，node kind=1/link kind=2）．割当: transferシャッフル・合流抽選・generate時経路選択＝nodeストリーム，走行中リンク端の経路選択・adj旅行時間ノイズ＝linkストリーム．route_next_link_choiceはRNG参照を引数で受ける形に変更
- 共有状態排除: vehicles_living/running削除（読み出しゼロを全数監査で確認），trips_completed_count→per-link部分和，ave_v_sum/ave_vratio_sum/stat_sample_count→RUNはper-link・WAITはper-node部分和（id昇順リダクション，bindingsはdef_prop_ro化），incoming_vehicles→`Link::arrived_vehicles` per-link収集バッファ＋transfer冒頭で車両id昇順sort集約（requests は route_next_link から再構築）
- ゲート: det_suiteグループA 7/7ビット同一，グループB 3/3 FAIL（RNG実現値変化，想定通り），stat_suite OVERALL PASS（heavy ttt p=0.523/signal ttt p=0.373，cross≒within），テスト211件通過（調整なし），ベンチ非退行（インターリーブ3ラウンド median-of-medians +0.58%，ノイズ内）
- 新リファレンス `baseline_det_aos_p1.json` 記録（自己一致10/10）
- Phase 2への申し送り: ホットパスの統計加算（`link_ave_v_sum[link->id] +=`）はRUN per-link化の際にリンクローカルへホイストしてflush一回化するのが自然．`Link::arrived_vehicles`はRUNパス（link専有）がpush・transferがdrainの構造で並列化時も競合なし
