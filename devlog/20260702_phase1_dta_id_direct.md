# Round 5 Phase 1 完了報告: DTA経路データのID直結化

実施: 2026-07-02（コミット `8eda4e2`）
計画: `20260702_plan_cpp_optimization_round5.md` Phase 1

## 実施内容

| Step | 内容 |
|---|---|
| 1-C | ODキーを `std::map<pair<int,int>>` から `ODPairHash` 付き `ODRouteSetStore`（unordered_map）へ。route_swapは点参照のみで順序非依存を確認 |
| 1-B | 経路集合をsolveごとに1回だけC++に構築し、ハンドルで `route_swap_due/dso` に渡す方式へ。毎iterのPython dict→C++ map再変換を廃止 |
| 1-A | `enumerate_od_route_sets_ids_cpp(k, seed)` 新設。同一のDijkstra列挙（同一rng順序）でC++常駐ストアを直接生成し、`nb::str`/`nb::dict` 生成を排除。名前版dictは `_LazyODRoutes` で遅延構築（GA/ALNSの解再利用パスでのみ実体化）。公開API `enumerate_k_random_routes` は不変 |
| 1-D | `route_swap_due/dso` の返却をflat CSR numpy（values+offsets）化。次iterの経路指定は新設 `batch_enforce_routes_csr` で直接返し込み、name↔id往復を全廃 |

## 検証

- ビット同一: `devlog/verify_bit_identity_dta.py`（9x9グリッド DUE+DSO 10iter、TTT列・最終経路・コスト・traveltime_real を完全一致比較）で全Step PASS
- テスト: test_cpp_mode.py 199 passed（reruns 5）、DTAサブセット25 passed、Pythonモード test_verification_dta_solvers.py 2 passed

## 効果（OMP_NUM_THREADS=1, 9x9, deltan=10, tmax=4800, 10iter, 5回中央値）

| Solver | Before | After | 削減 |
|---|---|---|---|
| DUE | 2.092s | 1.643s | 21.5% |
| DSO | 2.131s | 1.778s | 16.5% |

## 知見

- 計画時プロファイルの「nb::dict構築34%」は過大評価で、実体は大半が純C++のDijkstra列挙計算（0.840s≒36%）。py-spyのスタック集計がdict構築とDijkstraを混同していたと推定
- 削除可能だった名前変換・毎iter変換のオーバーヘッドは実測15〜20%であり、これは全て除去済み
- 残る最大ボトルネックのDijkstra列挙はPhase 3（3-b/3-c: バッファ再利用・密行列再確保排除）で対応
- `_build_name_id_maps` 等3関数が未使用化（残置）。旧 `batch_enforce_routes` bindingは汎用APIとして温存
