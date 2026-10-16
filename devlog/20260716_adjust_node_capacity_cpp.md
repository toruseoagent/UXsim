# adjust_node_capacity オプションのC++移植（2026-07-16）

## 概要

本家コミット785ccef（`add adjust_node_capacity option`）で追加されたPython版新機能をC++エンジンに移植した。

- `World(adjust_node_capacity=True)` で，`flow_capacity`未指定の全ノードの流量容量を接続リンク容量から自動決定する
- 計算式：`flow_capacity = (max(inlink容量) + max(outlink容量)) / 2`，車線数未指定なら `ceil(flow_capacity / 0.8)`

## 変更ファイル

| ファイル | 変更内容 |
|---------|---------|
| `uxsim/trafficpp/traffi.h` | `Node::adjust_node_capacity()` 宣言，`World::adjust_node_capacity` フラグ追加 |
| `uxsim/trafficpp/traffi.cpp` | `Node::adjust_node_capacity()` 実装．`World::initialize_adj_matrix()` 冒頭でフラグが立っていれば全ノードに適用 |
| `uxsim/trafficpp/bindings.cpp` | `World.adjust_node_capacity`（def_rw），`Node.adjust_node_capacity`（def）バインディング追加 |
| `uxsim/uxsim_cpp_wrapper.py` | `CppWorld.__init__` に引数追加・C++へ伝播，`CppNode.adjust_node_capacity()` メソッド追加（C++呼出し＋属性同期），`finalize_scenario` でラッパー側ノード属性を同期 |
| `tests/test_cpp_mode.py` | `test_verification_node.py` の新テスト3件を移植（`World(cpp=True, ...)`） |

## 実装メモ

- C++側の未指定`flow_capacity`は-1.0（Pythonの`None`に対応）なので判定は `flow_capacity < 0.0`
- `flow_capacity_remain = flow_capacity * delta_t` はPythonの `flow_capacity * DELTAT` と同一
- `flag_lanes_automatically_determined` はラッパー側で車線数の変化を検出して設定
- 適用タイミングは `initialize_adj_matrix()`（＝finalize時）でPython版の `finalize_scenario` 内適用と一致．`flow_capacity >= 0` になった後は再呼出ししても無害（冪等）

## 検証結果

- 新テスト3件（merge容量0.8 / major-minor自由流1.6 / major-minor渋滞1.6）：Python版期待値と同一のアサーションでC++モード通過
- 妥当性検証：6x6グリッド（主要道2車線・従属道1車線混在，需要8000台）5seedで，全ノードの`flow_capacity`・`number_of_lanes`が両モード完全一致．TTTは5seed平均で差+0.89%（seed毎のばらつきは乱数列の違いによる既知の挙動）
- リグレッション：`tests/test_cpp_mode.py` **217件通過**（threads引数のWorld定義化と結合したツリーで実施，flaky 6 rerun，407s）．`test_other_functions.py` 28件通過（1failはosmnx未導入の環境問題で無関係）
- ベンチマーク：ユーザー指示により省略（本機能はfinalize時のみのフラグガード付きコードでホットパス外）

## コミット・PR

- コミット c0fcae0 "Support adjust_node_capacity option in C++ mode"（threads引数の定義化 0d6b314 と分離してコミット）
- threads定義化と合わせて1本のPRとして本家へ送付（ユーザー指示「完成したらまとめてPR送って」「ベンチ不要．実装完了次第さっさとPR送って」）
