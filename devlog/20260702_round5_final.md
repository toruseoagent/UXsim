# Round 5 最終報告: C++エンジン最適化（計画〜完了）

実施: 2026-07-02
計画: `20260702_plan_cpp_optimization_round5.md`
フェーズ報告: `20260702_phase1_dta_id_direct.md` / `20260702_phase2_traveltime_lazy.md` / `20260702_phase3_small_optimizations.md`

## コミット

| コミット | 内容 |
|---|---|
| `8eda4e2` | Phase 1: DTA経路データのID直結化（CSR flat・C++常駐・unordered_map） |
| `919ccb5` | Phase 2: traveltime_real の遅延materialize化（介入読み出し対応） |
| `76d3866` | Phase 3: route_search_allバッファ再利用・psumフラグ化ほか小粒改善 |

変更はC++エンジン側（uxsim/trafficpp/）を主とし、Python側はDTAsolvers.py/wrapperの新API呼出切替のみ（ユーザー承認済み）。uxsim.py・analyzer.py・Utilities.pyは無変更。

## 最終精密ベンチマーク

条件: OMP_NUM_THREADS=1、アイドル環境、Round 5前（`5f1a998`）とHEADを同一マシン状態で背中合わせ計測。コアシム=11x11グリッド deltan=3 tmax=7200（3seed×5回）、DUE/DSO=9x9グリッド deltan=10 tmax=4800 10iter（各7回）。中央値±std。

| ベンチ | Round 5前 | Round 5後 | 削減 |
|---|---|---|---|
| コアシム seed=42 | 1.246s (±0.076) | 1.177s (±0.094) | 5.5% |
| コアシム seed=1 | 1.088s (±0.040) | 1.032s (±0.118) | 5.1% |
| コアシム seed=2 | 1.280s (±0.394) | 1.007s (±0.123) | 21.3%（前計測ノイズ大） |
| DUEソルバー | 1.983s (±0.191) | 1.495s (±0.134) | **24.6%** |
| DSOソルバー | 2.154s (±0.060) | 1.595s (±0.112) | **26.0%** |

計測ログ: `tmp/bench_final_r5_before.txt` / `tmp/bench_final_r5_after.txt`（git管理外）

## 検証

- **ビット同一性**: 全フェーズで変更前C++と完全一致（DUE/DSOのTTT列・最終経路・コスト、コアシムの全車両ログ・全リンクtraveltime_real、チャンク分割実行の各境界での介入読み出し、gradualモード、DSO外部性計算経路）
- **テスト**: test_cpp_mode.py **200 passed**（4 rerun、flaky既知）を最終ビルドで確認
- **Python等価性**: ビット同一のため既存検証を継承。加えてスポットチェックでC++/Python TTT差 +0.68%（9x9グリッド）

## 総括と残課題

- DTAソルバーはRound 5全体で約25%高速化（既存の8-12x比でさらに上乗せ）。主因はname↔id往復の全廃とDijkstra周りのアロケーション削減
- コアシミュレーションは約5%高速化（traveltime_real遅延化）。ユーザー介入（チャンク境界での読み出し）にも同一結果で対応し、最悪ケースでも旧実装と同等コスト
- 残課題（今回見送り・記録のみ）:
  - enumerate内の純Dijkstra計算が依然DUEの最大コスト（アルゴリズム自体の改良が必要な領域）
  - シム後ログ→numpy抽出（array_concatenate系）はRound 4放棄履歴があるためmeasure-first条件付き
  - リンク側traveltime系vectorのrealloc（3-f残余）
  - DUE毎iterのanalyzerオーバーヘッド（Python本体のためスコープ外）
