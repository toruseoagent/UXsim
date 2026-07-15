# Round 5 Phase 3 完了報告: 小粒改善（3-a〜3-f）

実施: 2026-07-02（コミット `76d3866`）
計画: `20260702_plan_cpp_optimization_round5.md` Phase 3

## 実施内容

| 項目 | 内容 | 結果 |
|---|---|---|
| 3-b | `route_search_all()` の隣接リスト・dist・next_hop・visited をWorldメンバのスクラッチバッファとして再利用（毎回のV×V確保を排除）、行参照キャッシュで二重間接参照削減。近傍集合・push順序・priority_queueタイブレークは不変 | 実施 |
| 3-c | `enumerate_k_random_routes_cpp` のwhileループ内V×V密行列確保をループ外へ（毎反復で全セル上書きのため値同一）。乱数呼出順序不変 | 実施 |
| 3-a | `route_choice_duo` / `route_choice_duo_gradual` のper-dest O(links) psum再計算を `route_pref_active` フラグに置換。route_preferenceは初期化以外で再ゼロ化されないことをコード確認済みで、psum==0判定と厳密に等価 | 実施 |
| 3-d | `dta_get_traveled_route` に reserve | 実施 |
| 3-e | `get_node`/`get_link` を既存 nodes_map/links_map のO(1)参照に（エラー時挙動不変） | 実施 |
| 3-f | 車両ログ6配列は既に `total_timesteps+10` でreserve済みと確認 → 変更なし。プロファイルのrealloc残余はリンク側 traveltime_t/tt/real_events 系の可能性が高いが、長さ予測が難しく過剰確保リスクを避けて見送り | 見送り |

## 検証

- ビット同一: 3系統すべてPASS — `tmp/verify_bit_identity_dta.py`（DUE/DSO）、`tmp/verify_bit_identity_core.py`（一括／チャンク介入読み／DSO外部性）、新規 `tmp/verify_bit_identity_gradual.py`（既存2系統は gradual パスを通らないため、route_choice_update_gradual=True の8x8グリッドで新設）
- テスト: test_cpp_mode.py 200 passed（8 rerun、flaky既知）

## 効果（OMP_NUM_THREADS=1, taskset固定, 9回中央値, HEAD⇔変更後を交互2ラウンド）

| ベンチ | 変化 |
|---|---|
| DUE 9x9, deltan=10, tmax=4800, 10iter | 約8%短縮（1.56-1.59s → 1.41-1.48s） |
| enumerate単体（k=10） | 約7%短縮（3-b/3-c寄与） |
| コアシム 11x11 | 中立（route_search_all呼出頻度が低くノイズ範囲内、想定通り） |

## 申し送り

- 変更は uxsim/trafficpp/ 3ファイルのみ（`route_search_all` はbindings非公開のためシグネチャ変更の外部影響なし。呼出3箇所を追従済み）
- 3-fのrealloc残余（リンク側traveltime系push_back）は離脱イベント数の見積もりが必要なため未着手として記録
- 確定性能値は最終フェーズの精密ベンチ（アイドル環境）で取得する
