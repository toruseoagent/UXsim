# DTAsolversリファクタ完了報告: C++向けコードのアダプタ分離

実施: 2026-07-03

## 内容

`uxsim/DTAsolvers/DTAsolvers.py` に混在していたC++エンジン向けコード（CppSolverDUE, CppSolverDSO_D2D と、id↔name変換・CSR経路処理・軽量finalize等のヘルパー、計770行）を、新規モジュール `uxsim/DTAsolvers/DTAsolvers_cpp_adapter.py`（789行）へ分離した。

コア `DTAsolvers.py` に残るC++への言及は以下の4行のみ:
- `SolverDUE.__new__` / `SolverDSO_D2D.__new__` 内の cpp=True 分岐での**遅延import** 各1行
- `__init__` 先頭の `_is_cpp_solver` ガード各1行（Cppソルバーは自身の__init__で初期化するため）

遅延import設計により、純Pythonモードの利用ではadapterモジュール（およびC++拡張）は一切読み込まれない。ロジック変更なしの純粋な移動。

## 検証

- ビット同一（`tmp/verify_bit_identity_dta.py`、DUE/DSO 10iter）: **PASS**
- `from uxsim.DTAsolvers import SolverDUE` 直後に adapter が sys.modules に載らないこと: 確認
- Pythonモード: test_verification_dta_solvers.py **4 passed**
- C++モード: test_cpp_mode.py **200 passed**（6 rerun、flaky既知）

## 経緯メモ（運用）

実装エージェントがpytestをバックグラウンド起動→出力ファイルをRead連続ポーリングする高速ループに陥ったため停止・廃棄し、残りの検証（ビット同一比較・テスト実行）は指揮官が直接実施した。対策はメモリに記録済み（長時間コマンドは同期フォアグラウンド実行を義務付け、バックグラウンド出力ファイルのポーリング禁止）。
