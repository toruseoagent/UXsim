# Round 5 Phase 2 完了報告: traveltime_real の遅延materialize化

実施: 2026-07-02（コミット `919ccb5`）
計画: `20260702_plan_cpp_optimization_round5.md` Phase 2

## 実施内容

- `Node::transfer()` / `Vehicle::end_trip()` でのリンク離脱イベントごとの `traveltime_real[start_idx..末尾]` スカラー埋め（大規模シナリオで数億回書き込み）を、`(start_idx, tt)` イベントのバッファ追記（O(1)）に置換
- `Link::ensure_traveltime_real()` が初回読み出し時に未反映イベントを再生: イベントkは区間 `[s_k, min(s_{k+1}, n))`、最後の未反映イベントのみ末尾まで埋める。「各イベントが `[s_k, 末尾]` を順に上書き」する従来動作の合成結果と等価（start_idxの単調性に依存しない。位置iの最終値=「s_k≤iを満たす最後のイベント」で両者一致）
- 読み出し全入口に ensure を挿入: bindings の `traveltime_real`/`traveltime_actual` プロパティ（def_ro→def_prop_ro化）、`get_traveltime_actual_np`、`dta_get_actual_travel_time`、`estimate_congestion_externality`（route版含む）
- ユーザー介入対応: C++モードのPython観測点は `exec_simulation(duration_t)` チャンク境界のみで、そこでの読み出しは必ずgetterを通るため任意タイミングでビット同一。読まなければmaterializeはシム後1回のみ、毎ステップ読む最悪ケースでも従来と同等コスト

## 検証

- ビット同一（`devlog/verify_bit_identity_core.py`）: シナリオA（一括実行: 全リンクtraveltime_real・全車両ログ・TTT）、B（600sチャンク分割＋各境界で全リンク読み出し=介入再現）、C（小規模DSO=externality経路）の3系統すべてPASS
- テスト: test_cpp_mode.py 200 passed（介入読み出しの回帰テスト `test_traveltime_actual_intervention_read_chunked` を新規追加）
- 注: 実装エージェントがセッション中断で停止したため、検証（参照再生成→比較→テスト→計測）は指揮官が実施

## 効果（OMP_NUM_THREADS=1, 11x11グリッド, deltan=3, tmax=7200, 5回中央値）

| Before | After | 削減 |
|---|---|---|
| 1.441s (std 0.47) | 0.968s (std 0.13) | 約33% |

ベースライン計測のばらつきが大きいため参考値。確定値は最終フェーズの精密ベンチで取得する。
