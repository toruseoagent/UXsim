# C++エンジンとPythonコアの精密比較と整合性修正

日付: 2026-07-11
関連PR: pr/cpp-python-consistency（本家へ送付）

## 目的

uxsim.py（コアシミュレータ）とC++エンジン（traffi.h/traffi.cpp/bindings.cpp/uxsim_cpp_wrapper.py）をエッジケース含めて1行対応で精密比較し，検出した差異のうち実害のあるものを修正する．

## 調査方法

- 両実装の全コアロジック（Node/Link/Vehicle/RouteChoice/Worldメインループ）をコード読解で突き合わせ
- 判断がつかない箇所は計算実験（`tmp/precise_compare/experiments.py`，git管理外）で両モードを同一シナリオ実行して比較

## 検出した差異と対応

| # | 差異 | 対応 |
|---|------|------|
| 1 | 自動TMAXが7200秒を超えるとC++が途中停止し `check_simulation_ongoing()` が永遠にTrue | `World::set_t_max()` 新設，finalize時にリサイズ |
| 2 | 自動TMAXの値が異なる（需要終了時刻ベース vs 最終出発時刻ベース） | `_compute_auto_tmax()` でPython準拠に |
| 3 | instantaneous_TT_timestep_intervalがC++に未伝達（常に5） | `_ensure_cpp_world` で伝達 |
| 4 | duo_update_time < DELTAT でC++は経路更新が走らない | C++側で最小1にクランプ |
| 5 | 需要・出発時刻の量子化差（DELTAT非整数倍で台数±1，出発時刻floor vs ceil） | add_demandを整数タイムステップループに |
| 6 | duration_t/duration_t2で1ステップ超過，TIMEの規約差 | exec_simulationをPython同一のend_ts計算に |
| 7 | addVehicleがNoneを返す，name/attribute等のkwargs無視 | Vehicleプロキシ返却，name/auto_rename/attribute対応 |
| 8 | 途中の `node.signal = [...]` 再代入が無効 | signal/cycle_lengthをC++転送プロパティ化 |
| 9 | 途中の `link.capacity_out/in` 直接代入が無効 | C++転送プロパティ化 |
| 10 | links_avoid全遮断・specified_route不整合でPythonは例外，C++は無言 | C++からValueError/IndexError送出 |
| 11 | log_s（車頭距離ログ）が常に0 | C++でリーダー車間距離を記録 |
| 12 | analyzer.average_speedが常に0（wait車両の0速度サンプル含む） | C++累積値をanalyzerに同期 |

修正見送り（ユーザー判断）: hard_deterministic_modeの最短経路タイブレーク差（scipy vs 自作Dijkstra，どちらも正しい最短経路），heterogeneous_DUO未実装，軽微な属性表現差（Node.number_of_lanesのNone/-1等）．

## 検証

- リグレッション: `tests/test_cpp_mode.py` 210件通過（新規一致テスト3件含む．links_avoidテスト1件はPython準拠のValueError期待に更新）
- 妥当性: 9x9グリッド・10530台・10シードでTTT平均差0.09%（シード別最大0.82%），完走数完全一致．標準シナリオでは累積曲線ビット単位一致
- ベンチ: OMP_NUM_THREADS=1・10シード，Python中央値2.079s vs C++ 0.137s（15.2倍）

## ノウハウ

- ラッパーの`_sync_from_cpp`は「Python名 = str(インデックス)」を仮定していた．カスタム名対応のため`_veh_by_index`（C++インデックス順のプロキシリスト）に置換．VEHICLES辞書の挿入順=インデックス順の不変条件はrename（pop+再挿入）が常に末尾要素に対してのみ行われることで保たれる
- C++からの例外はnanobindが自動変換する（std::invalid_argument→ValueError, std::out_of_range→IndexError）．Pythonと同じ例外型を意図的に選べる
- C++ Worldはワールド生成時にtotal_timestepsで配列を確保するため，TMAX確定がaddNode後になる場合はリサイズAPIが必須（プレースホルダ7200秒の罠）
