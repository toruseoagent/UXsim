# 完了報告: C++エンジンECS/SoA化リファクタと決定的並列化

> **【2026-07-16追記】このSoA化アプローチは不採用となった。**
> SoA化自体の1スレッド高速化効果が小さかったため，SoA化なしで同等の決定的並列化を実装した代替案（AoS維持，`devlog/20260715_plan_aos_parallel_engine.md`）が採用され，PR #341としてマージされた。本PR #340はクローズ。
> 本文書はその経緯・計測・ノウハウの記録として保存する（log_s判別則・RNGストリーム分離・検証ハーネス等はAoS版に流用された）。

実施: 2026-07-15
ブランチ: `refactor/ecs-soa-engine`（fork push済み，本家PR未送信・指示待ち）
計画書: `devlog/20260715_plan_ecs_soa_engine_refactor.md`（§8に各フェーズの経過記録）

## 成果の要約

C++エンジン内部をECSの核となる構成（整数ID＋SoAコンポーネント配列＋状態別システム関数）に全面移行し，その上でOpenMPによる決定的並列化を実現した。
外部インターフェース（bindings属性・ラッパー・analyzer連携）は完全互換で，**uxsim_cpp_wrapper.pyの変更はゼロ**，既存テスト211件は**無調整で全通過**。

| 指標 | 値 |
|---|---|
| 対Python（1スレッド, heavy 11x11） | **40.0x**（total中央値 81.9s→2.05s） |
| 対リファクタ前（1スレッド） | heavy exec −13.5%，total −14.5% |
| スレッドスケーリング（main_loop, ログON） | 8スレッドで **2.76〜2.88x**（1.31s→0.47s） |
| メモリ | RSS −75MB，VmPeak −417MB（heavy） |
| 再現性 | 同一シードなら**スレッド数に依らずビット同一**（1/2/8で確認） |

## 実装内容（コードコミット）

| コミット | フェーズ | 内容 |
|---|---|---|
| 68da00c | Phase 1 | Vehicle hot 8変数をWorld SoA配列へ移設，idx＋インラインアクセサのファサード |
| 1903ed7 | Phase 2 | リンク車両キューをint32リングバッファ＋単調seq番号に置換，leader/followerを位置導出化 |
| 902500f | Phase 3 | メインループの状態別システム化（RUN融合パス・WAITパス・HOME出発バケット・update_order廃止・incoming id順正準化） |
| 29e67b1 | Phase 5 | 固定費削減（route_preference lazy化，ログreserve右サイズ化，選択バッファ共有化） |
| e80b91d | Phase 7 | RNGをper-node/per-linkストリームに分離，共有可変状態の排除 |
| 1ecb30f | Phase 8 | OpenMP決定的並列化（単一parallelリージョン＋ステージ別for，transferはsingleで直列） |
| 4d2be00 | Phase 9 | `World(cpp=True, threads=1)` 引数追加（-1で全コア，wrapperにcppモード専用拡張，テスト212件目追加） |

Phase 4（ログのtimestep-majorアリーナ化）は実装・計測の結果**不採用・revert**（ログコストの本質は書き込み量であり配置ではないと実証。Round 4の教訓を再確認）。

## 検証（Phase 0ハーネス＋Phase 6総合検証）

- **ビット同一性**: 乱数非依存シナリオ7種は全フェーズを通じて旧実装とビット同一。乱数消費順序が変わったPhase 3・7では30シードの統計的同等性で検証（Welch p>0.05，リンク相関cross≧within）
- **Python妥当性**（Python 10シード vs C++ 30シード）: heavy/signal全指標でp=0.11〜0.86・平均差1.8%以内，リンク交通量cross相関はwithin同水準。OVERALL PASS
- **スレッド数非依存**: OMP_NUM_THREADS=1/2/8で全10シナリオ（確率的含む）ビット同一
- **リグレッション**: test_cpp_mode.py 211件通過（最終確認375s）。--cppフラグの既存Python版テストも通過（straight_roadの3件failは既知のlist+numpyハーネス非互換で，エンジン回帰ではない）

## 設計上の主要な発見・判断（詳細は計画書§8）

1. **1スレッド・レイアウト変更のみでは当初目標（main_loop 1.5〜2.5x）に到達不能**と計測で結論（Phase 4完了時）。死に走査除去は本家が既にP1で取り込み済み，残る支配項のログ書き込みは「量」が本質でbaseも同額を払う。ユーザー承認のもと決定的並列化へ方向転換し，8スレッド2.9倍で当初目標水準を達成
2. **log_sの旧セマンティクスはid順走査に依存**（leaderのidが自車より小さければ移動後のx，大きければ移動前のx）。Phase 3で判別則により再現
3. **transferは並列化しない**: 直列ノードid順の逐次可視性セマンティクス（Python版と同一）を持つため。`omp single`で維持
4. **scikit-build-coreのビルドはリポジトリ直下のCMakeLists.txtを使用**。OpenMPフラグは直下側に必要

## 残課題（スコープ外として記録）

- exec_simulation全体のマルチスレッドスケーリング（8Tで1.9x）は直列のPython後処理（`_sync_from_cpp`約0.6〜0.7s）が上限要因。後処理の削減・並列化は将来課題
- log_heavyのエクスポート経路（logbuild）は本リファクタの対象外（変化なし）
- 検証ハーネス（tmp/ecs_refactor/: det_suite, stat_suite, bench_suite, scale_bench, python_parity, bench_py_vs_cpp）はgit管理外。今後の変更検証に再利用可能

## PRについて

未送信（ユーザー指示待ち）。送信時の注意: コードコミット6個をorigin/mainベースのPRブランチにcherry-pick（devlog等フォーク専用ファイルは除外），ルートCMakeLists.txtの変更を含めること，PR本文に上記ベンチ表を記載。
