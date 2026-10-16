# Node.override_signal() のC++モード対応

日付: 2026-07-14

## 背景

本家PR #338（2026-07-14マージ）で `Node.override_signal(signal, groups=None, signal_offset=0, reset=False, signal_offset_old=None)` がPython版に追加された．
信号設定を後から上書きするセットアップ用APIで，`groups` 指定時は各リンクの `signal_group` を書き換える．

## 実装

- `uxsim/uxsim_cpp_wrapper.py` の `CppNode` に `override_signal()` を追加．Python実装の1行ずつ移植
- C++本体（traffi.h/cpp, bindings.cpp）の変更は不要．
  `signal_intervals`/`signal_phase`/`signal_t`/`signal_offset`（Node）と `signal_group`（Link）は既に `def_rw` でバインド済みであり，シミュレーションループはこれらを直接参照するため，ラッパーからの書き換えがそのまま反映される
- 注意点: `CppLink.signal_group` は平属性（init時のみC++へ転送）なので，`override_signal` 内で `_cpp_link.signal_group` への同期も行う
- `tests/test_verification_node.py::test_override_signal` の `assert cum1 == cum2` は cppモードでnumpy配列となり `--cpp` 実行で失敗するため，`list()` 比較に修正（両モード互換）

## 検証

- リグレッション: `tests/test_cpp_mode.py` 211件全通過（flaky rerun 9件は成功）
- 新テスト: `test_cpp_mode.py::test_override_signal`（Python版テストのcpp移植），`test_verification_node.py::test_override_signal`（素・`--cpp`両方）通過
- エラー系（グループ長不一致・非流入リンク）のValueErrorメッセージもPython版と同一
- 妥当性: mergeシナリオ（deltan=1, tmax=3600, 3 seed）でPython vs C++のTTT差0.49〜4.06%．各モード内でinitial設定とoverride設定のTTT・累積カーブが完全一致
- ベンチ: 信号付き8x8グリッド（override_signalで信号設定，deltan=5, tmax=3600, OMP_NUM_THREADS=1, 10 seed）
  - Python: median 22.670s (std 1.076s)，C++: median 0.719s (std 0.108s)，**31.5x**
  - TTT中央値差+13.79%はシード間ばらつき由来（Welch t検定 p=0.454，有意差なし）

## 成果物

- コミット: `Add Node.override_signal() support to C++ mode`
- ベンチスクリプト: `tmp/bench_override_signal_pr.py`，検証スクリプト: `tmp/validate_override_signal.py`（git管理外）
