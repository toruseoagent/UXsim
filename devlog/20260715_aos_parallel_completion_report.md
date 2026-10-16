# 完了報告: 非SoA（AoS維持）でのC++エンジン決定的並列化

実施: 2026-07-15
ブランチ: `refactor/aos-parallel-engine`（fork mainベース，本家PR未送信・指示待ち）
計画書: `devlog/20260715_plan_aos_parallel_engine.md`（§6に各フェーズの経過記録）

## 成果の要約

fork mainのAoSエンジン（Vehicle/Link/Nodeクラス・deque・leader/followerポインタを維持，**SoA化なし**）に，SoA版（PR #340）と同等の決定的OpenMP並列化を実装した。
外部インターフェースは完全互換（wrapperの変更はthreads引数の追加のみ，既存テスト211件は無調整で通過）。

| 指標 | 値 |
|---|---|
| 対Python（1スレッド, heavy 11x11, exec中央値） | **40.7x** |
| 対Python（8スレッド, 同上） | **56.5x** |
| 対fork main（1スレッド, heavy exec） | **−10.2%**（2.136→1.918s, 1.11x） |
| スレッドスケーリング（main_loop, ログON） | 2T 1.92x / 4T 2.92x / **8T 3.69x** |
| 再現性 | 同一シードならスレッド数（1/2/8）に依らず**ビット同一** |
| Python妥当性（10 vs 30シード） | 全指標 p=0.11〜0.86，平均差≤1.8%，cross≥within。PASS |
| リグレッション | test_cpp_mode.py 212件通過（threads回帰テスト1件追加） |

## SoA版（refactor/ecs-soa-engine, PR #340）との比較

| 指標 | SoA版 | 本AoS版 |
|---|---|---|
| 1スレッド（対リファクタ前/main） | −13.5% | **−10.2%** |
| main_loop 8スレッド | 2.76〜2.88x | **3.69x** |
| データレイアウト変更 | 全面SoA化（7コミット） | **なし**（4コミット） |
| 確率的シナリオのビット互換（対main） | 変化（Phase 3/7で乱数順変更） | Phase 1（RNG分離）でのみ変化 |

並列スケーリングはAoS版が全スレッド数で上回った。仮説: AoSはRUNパスの1スレッド並列化可能フラクションが相対的に高く，直列transferの比率が小さいためAmdahl的に有利。
1スレッド性能もSoA版とほぼ同水準を，ドラスティックなレイアウト変更なしで達成した（主因はPhase 2の融合RUNパスによる簿記除去とdeque連続走査の局所性）。

## 実装内容（コードコミット4個）

| コミット | フェーズ | 内容 |
|---|---|---|
| beddd7e | Phase 1 | RNGのper-node/per-linkストリーム分離，共有可変状態の排除（統計部分和化・incoming per-link収集バッファ化・vehicles_living/running削除） |
| bd7d434 | Phase 2 | メインループのステージ分割（RUN融合パス per-link・WAIT per-node・HOME出発バケット，log_s判別則によるid順走査セマンティクスのビット単位再現） |
| 9071af8 | Phase 3 | 決定的OpenMP並列化（単一parallelリージョン＋ステージ別for，transferはsingle，thread_localスクラッチ，例外の最小id優先再送出，ルートCMakeLists.txtにOpenMP） |
| 122aef8 | Phase 4 | `World(cpp=True, threads=1)` 引数（デフォルト1，-1で全コア） |

## 検証（Phase 5総合検証）

- **ビット同一性**: 乱数非依存7シナリオは全フェーズを通じてfork mainとビット同一。確率的3シナリオはPhase 1（RNGストリーム分離）でのみ実現値が変化し，30シード統計的同等（stat_suite全PASS）で検証。Phase 2のステージ分割は全10シナリオでビット同一を維持（リンク当りRNG消費者が先頭到達車1台のみのため消費順不変）
- **スレッド数非依存**: threads=1/2/8で全10シナリオビット同一
- **Python妥当性**（python_parity, Python 10シード vs C++ 30シード）: heavy/signal×TTT/完了数/平均速度の全てで p>0.05・平均差1.8%以内，リンク交通量cross相関はwithin水準。OVERALL PASS
- **リグレッション**: test_cpp_mode.py 212件通過。test_verification_straight_road.py --cpp は既知の3件fail（list+numpyのテストハーネス非互換，エンジン回帰ではない）以外全通過

## 設計上の主要な発見・判断

1. **SoA化は並列スケーリングの前提条件ではなかった**。並列化の実体（RNGストリーム分離・共有状態排除・ステージ分割・決定的リダクション）はデータレイアウトと独立に移植可能で，スケーリングはむしろAoSが有利だった
2. **1スレッド高速化の主因もレイアウトでなく走査構造**。Phase 2の融合RUNパス（car_follow別パス廃止・update_order簿記除去・per-link deque連続走査）だけで−14.8%（インターリーブ計測）を達成。SoA版Phase 4不採用の教訓「ログコストは配置でなく量」と整合
3. **log_sのid順走査依存はSoA版Phase 3の判別則（`leader_id < self_id ? 移動後x : x_old`，同step終了leaderは-1）がAoSでもそのまま有効**。キュー位置導出leader（snapshot[i-lanes]）でend_trip後も安全に参照できる
4. **exec全体の8Tスケーリング（1.61x）は直列のPython後処理（_sync_from_cpp）が上限**（SoA版と同じ既知の残課題）

## 残課題（スコープ外として記録）

- exec_simulation全体のスケーリング向上にはPython側後処理の削減・並列化が必要（SoA版と共通の課題）
- update_adj_time_matrixは直列維持（SoA版踏襲）
- 検証ハーネス（tmp/ecs_refactor/）はgit管理外。本フェーズで追加した使い捨てスクリプト: ab_interleave_p5.sh, bench_cpp8_vs_py.py
- CLAUDE.mdのbindings説明は「pybind11」だが実際は両CMakeともnanobind（要修正検討）

## PRについて

**PR #341として送付し，2026-07-16にマージされた（squash 30285b9）。SoA版PR #340はクローズ＝本AoS実装が採用**。
送付後の経緯: ユーザー懸念（Python構造との乖離）を受けPhase 6（折衷変種: Vehicle::update/car_follow_newell/living・runningレジストリの復元）を追加採用し，PR本文を更新（1Tは対mainパリティ，対Python 1T 28.2x/8T 49.7x，mainloop 8T 3.90x）。ユーザー追加のVEHICLES_RUNNINGアクセステストを含む214件通過を確認．本家main進行（#342/#344）に伴うrebaseを経てマージ。
