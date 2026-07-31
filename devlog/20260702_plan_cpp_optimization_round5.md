# C++エンジン最適化 Round 5 計画

作成: 2026-07-02（Fable 5指揮、Opus 4.6エージェント3体による分析に基づく）

## 前提・制約

- **変更対象はC++側のみ**（`uxsim/trafficpp/` 配下）。新C++ APIへの呼び出し切替のための最小限の変更が `uxsim/DTAsolvers/DTAsolvers.py`・`uxsim/uxsim_cpp_wrapper.py` に必要な項目は明記し、事前承認を得る
- **機能・計算結果は完全維持**。各項目に「結果同一性」欄を設け、ビット同一を保てるか明記
- 浮動小数点演算順序・乱数消費順序・車両処理順序を変える変更は不採用
- Round 4で放棄済みの項目（LogEntry AoS化、GIL release、full flat log API、_traveled_nodes事前確保）は再提案しない
- 車両更新ループは非決定モードでID順必須（`20260402_fix_vehicle_update_order.md`）— 触らない

## 現状ベースライン（2026-07-02計測、OMP_NUM_THREADS=1、概算）

| シナリオ | 中央値 |
|---|---|
| コアシミュレーション（11x11グリッド, deltan=3, tmax=7200） | 3.22s |
| DUEソルバー（9x9グリッド, deltan=10, tmax=4800, 10 iter） | 2.99s |

### プロファイル要点（py-spy --native, 各2.2万サンプル超）

**DUEソルバー**（シミュレーション本体は約15%しかない！）:
- `enumerate_k_random_routes` 41% cum — うち **`nanobind::dict` 構築だけで34% cum**（C++のIDを文字列化してPython dictにして返し、DTAsolvers側でIDに戻している往復が主因）
- `std::map<pair<int,int>>` 赤黒木比較 ~11% cum（ODキーのordered map）
- Dijkstra `route_search_all` 19.8% cum、`actual_travel_time` 16.6% cum
- analyzer毎iterオーバーヘッド ~17% cum（Python側、今回のスコープ外）

**コアシミュレーション**:
- シミュレーション中のログ書込 ~17% cum（うち `vector::push_back` realloc 7.8%）
- シミュレーション後のログ→numpy抽出 ~21% cum（`array_concatenate` の準二次的コスト含む）
- `Node::transfer` 10.8% cum（内部に `traveltime_real` スライス埋めを含む）、car-follow 4.6%

---

## Phase 1: DTA経路データのID直結化【最優先・効果最大】

### 1-A. enumerate_k_random_routes のID返却API新設
- **現状**: `enumerate_k_random_routes_cpp`（traffi.cpp:1189）は内部でリンクIDのまま経路を生成 → bindings.cpp:379-397 が全経路を `nb::str`（ノード名・リンク名）に変換してPython dict構築（**DUE全体の34%**）→ Utilities.py で `list()` 再コピー → DTAsolvers.py `_convert_od_routes_to_ids` で**IDに戻す**
- **提案**: `enumerate_k_random_routes_ids_cpp(k, seed)` を新設し、**IDのままflat CSR形式（int numpy配列＋offsets配列）で返却**。`nb::dict`/`nb::str` 生成を全廃
- **呼び出し側**: CppSolverDUE/CppSolverDSO_D2D が新APIを直接使用（DTAsolvers.py の呼出切替が必要）。名前版 `enumerate_k_random_routes`（Utilities.py経由の公開API）は現状のまま温存
- **結果同一性**: 乱数はseed引数で従来通り → 消費順序不変。ノード/リンクIDは `func_World()` が決定論的に再構築するため各iterで安定（既存コードも暗黙に依存済み）。**同一経路集合になることを検証必須**
- **難易度**: 中／**変更**: bindings.cpp, traffi.cpp（必要なら）, wrapper（薄いメソッド）, DTAsolvers.py（呼出切替）

### 1-B. od_route_sets のC++側常駐化
- **現状**: bindings.cpp:545-561（DUE）/590-606（DSO）が**毎イテレーション**同一の od_route_sets_ids（数万経路）を Python dict → `std::map<pair<int,int>, vector<vector<int>>>` に再変換
- **提案**: 経路集合を一度だけC++に登録するAPI（Worldまたはソルバー構造体にキャッシュ）。route_swap は登録済みデータを参照。1-Aと組み合わせれば「C++で生成→C++に常駐→C++で消費」となりPythonを経由しない
- **結果同一性**: データ同一・演算順序不変 → ビット同一
- **難易度**: 中／**変更**: bindings.cpp, dta_solver.h/.cpp, DTAsolvers.py

### 1-C. ODキーの std::map → unordered_map（またはdenseインデックス）
- **現状**: `std::map<std::pair<int,int>,...>`（dta_solver.h:100,109ほか）。赤黒木比較がDUEの~11% cum
- **提案**: pairハッシュ付き `unordered_map`、または `o*n_nodes+d` のdense配列インデックス
- **結果同一性**: 各車両独立のlookupで探索順序は結果に無関係 → ビット同一
- **難易度**: 低／**変更**: dta_solver.h/.cpp（＋enumerate内のmap）で完結

### 1-D. route_swap結果のflat numpy返却
- **現状**: bindings.cpp:564-582/611-628 が車両ごとに `nb::list` へ1要素ずつappend（車両数×リンク数のPyObject生成、大規模で線形悪化）
- **提案**: 既存の `build_all_vehicle_logs_flat_compact`（bindings.cpp:489-505）と同じ flat配列＋offsets 方式
- **結果同一性**: 値不変 → 同一
- **難易度**: 中／**変更**: bindings.cpp, DTAsolvers.py（_convert_cpp_result_to_names）

**Phase 1 期待効果合計: DUEソルバー全体で30〜40%短縮**（enumerate 41%の大半＋map操作11%の大半＋route_swap変換分）

## Phase 2: コアシミュレーション — traveltime_real のバッチ構築

- **現状**: `Node::transfer()`（traffi.cpp:304-312）と `Vehicle::end_trip()`（764-772）が、リンク離脱イベントごとに `traveltime_real[start_idx..末尾]` をスカラーループで埋め直す。イベント数×平均フィル長で**数億回規模の書き込み**。Python版はnumpyスライス代入なので安価だがC++では顕在化
- **検証済み事実**: `traveltime_real` はmain_loop実行中は**write-only**（C++内の読み出しは `estimate_congestion_externality` と `dta_solver.h::get_actual_tt` の2系統＋Python公開のみ、grep確認済み）
- **提案（遅延materialize方式）**: イベント `(start_idx, tt)` をリンクごとに追記するだけにし、**読み出し時に未反映イベントを一括反映**する
  - bindingsの `traveltime_real` を getter付きプロパティに変更し、アクセス時に `ensure_traveltime_real(link)` を実行してから返す → **ユーザーがexec_simulationチャンク境界で介入して読んでも現行と同一の値が見える**（C++モードはuser_functionコールバック非対応のため、Pythonからの観測点はチャンク境界のみ）
  - C++内部の読み出し2系統の先頭でも同じ `ensure` を呼ぶ
  - materializeは未反映イベントを順に「区間 `[s_k, s_{k+1})` を埋め、最後のイベントのみ末尾まで埋める」方式で再生。「各イベントが末尾まで上書き」する現行の合成結果と等価で、start_idxの単調性に依存しない
- **結果同一性**: 単純代入の再構成でありfloat演算なし → 任意の観測タイミングで**ビット同一**
- **性能特性**: 介入なしなら materialize はシム後1回のみ（効果最大）。毎ステップ読む最悪ケースでも現行実装と同等コストで、**現行より遅くなるケースは存在しない**
- **期待効果**: コアシミュレーション5〜10%
- **難易度**: 中／**変更**: traffi.h/.cpp のみ

## Phase 3: 小粒の確実な改善

| # | 項目 | 箇所 | 効果 | 同一性 | 難易度 |
|---|---|---|---|---|---|
| 3-a | gradualモードのper-dest psum再計算を初期化フラグで除去 | traffi.cpp:1311-1338 | gradualモードで数% | ビット同一 | 低 |
| 3-b | `route_search_all` の隣接リスト・dist/visited バッファ再利用 | traffi.cpp:1135-1187 | DTA経路列挙・大規模で中 | ビット同一（Dijkstra決定的） | 中 |
| 3-c | `enumerate_k_random_routes_cpp` 反復内のV×V密行列再確保を排除 | traffi.cpp:1209-1219 | 低〜中 | 乱数呼出順を維持すれば同一 | 低 |
| 3-d | `dta_get_traveled_route` に reserve | dta_solver.cpp:12-30 | 低 | 同一 | 低 |
| 3-e | `get_node`/`get_link` の線形探索を既存 nodes_map/links_map 参照に | traffi.cpp:1514-1534 | 微小（方針整合） | 同一 | 低 |
| 3-f | 車両ログvectorのreserve状況を実測確認し、漏れがあれば追加 | traffi.cpp log_data系 | push_back realloc 7.8%の一部 | 同一 | 低 |

※3-f はレビュー担当（reserve済と判断）とプロファイル（realloc顕在）で見解が割れたため、**実装前にどのvectorが未reserveか特定する**こと。

## 保留・不採用

- **シム後ログ→numpy抽出（~21% cum）の改善**: `array_concatenate` の準二次コストは実測で確認されたが、Round 4で類似施策（full flat log API・遅延キャッシュ）が「効果なし・一部退行」で放棄済み。**再挑戦するなら log-heavy ベンチで単体measure-firstを条件とする**（今回は非採用）
- **analyzer毎iterオーバーヘッド（DUEの~17%）**: analyzer.py はPython本体で変更不可。スコープ外として記録のみ
- **OpenMP並列化（car_follow等）**: 結果ビット同一で並列化可能だが、PR規定ベンチが1スレッドのため数値に現れない。不採用
- **-march=native / FMA**: 浮動小数丸めが変わり結果非同一の恐れ。採用するならデフォルトOFFのCMakeオプトインだが、今回は非採用
- **-ffast-math, LTO**: 前者は結果が変わるため禁止。後者は単一translation unitのため無意味
- **SoA化**: 未実装のまま計画書だけ存在。Round 4のAoS失敗を踏まえ投機的 → 不採用
- **Round 4放棄項目全般**: 再提案しない

## 検証プロトコル（各Phase完了ごと）

1. **ビット同一性チェック**: 変更前後のC++で同一シナリオ（グリッド＋Sioux Falls、DTAはDUE/DSO）を実行し、車両ログ・TTT・traveltime_real が**完全一致**することを確認（ビット同一を約束した項目）
   - Phase 2は加えて**介入読み出しテスト**: `exec_simulation(duration_t=...)` で分割実行し、各チャンク境界で `traveltime_real` を読んで変更前C++と完全一致することを確認（test_cpp_mode.py にもテスト追加）
2. **リグレッション**: `pip install -e . && python3 -m pytest tests/test_cpp_mode.py --reruns 5 -q --tb=short`
3. **Python等価性**: Python版との TTT 数%以内一致（既存 validation スクリプト再利用）
4. **精密ベンチ**: OMP_NUM_THREADS=1、複数seed、中央値+std。**ベンチ中は他プロセス禁止**

## 実施順序

1. Phase 1（1-C → 1-B → 1-A → 1-D の順。依存が少ない順に積み上げ、各ステップでビット同一検証）
2. Phase 2（安全版の区間書き方式で実装 → ビット同一検証 → 必要なら単調性活用版に進む）
3. Phase 3（3-f の実測確認 → 低難易度項目を一括）
4. 最終検証＋精密ベンチ → PR準備

## 分析成果物

- プロファイル生データ・再利用スクリプト: scratchpad の `baseline.py`, `dta_bench.py`, `agg.py`, `core_raw2.txt`, `dta_raw2.txt`
- 注意: 過去の `devlog/timing_breakdown.txt`（convert_od_routes_to_ids 46%）は**陳腐化**（キャッシュ実装済みで現在0.03s）。devlogベンチスクリプトの一部は `solve(cpp=...)` という現存しないシグネチャを参照しており stale
