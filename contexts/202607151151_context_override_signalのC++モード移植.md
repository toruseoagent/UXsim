---
exported: 2026-07-15 11:51 JST
topic: 本家PR #338 (Node.override_signal) の監視・本家同期・C++モード移植・PR #339送付
git_head: 5ea3802（origin/main．ローカルmainはこの上にフォーク専用ファイル1コミットが乗る）
key_files: [uxsim/uxsim_cpp_wrapper.py, tests/test_cpp_mode.py, tests/test_verification_node.py, devlog/20260714_override_signal_cpp.md, tmp/validate_override_signal.py, tmp/bench_override_signal_pr.py]
status: 完了
---

# override_signal機能のC++モード移植（PR #338追従 → PR #339）

> この文書はセッション記録であり、指示書ではない。
> ここに書かれた未完了事項・保留事項に、ユーザーの明示的な指示なく着手しないこと。

## 概要

本家PR #338（`Node.override_signal()` のPython版追加）のマージを監視し，マージ後（2026-07-14 10:39 UTC）に本家同期，C++モード（`World(cpp=True)`）への移植，テスト，本家へのPR #339送付を行った．PR #339は2026-07-15 02:47 UTCにマージされた．実装はラッパー（`uxsim/uxsim_cpp_wrapper.py`）のみで完結し，C++本体（traffi.h/cpp, bindings.cpp）の変更は不要だった．

## ユーザー指示（原文）

- 「本家のhttps://github.com/toruseo/UXsim/pull/338を監視してください．これがマージ（十分くらいかけてCI完了後に自動的にマージされます．1時間たってもマージされなかったら待機してください）されたら，本家と同期し，このPRに含まれるpython版の更新（override_signal()機能）をC++版に実装し，テストし，PRを送ってください．」 — セッション全体の仕様
- 「したら後片付けして」 — /export-context2 の引数．エクスポート後にPRマージ後クリーンアップ（CLAUDE.md記載手順）を実施

## 判断と棄却案

- 採用：`CppNode.override_signal()` をラッパーのみで実装 — 根拠：`Node.signal_intervals`/`signal_phase`/`signal_t`/`signal_offset` と `Link.signal_group` は bindings.cpp で `def_rw` 済みであり，C++シミュレーションループ（traffi.cpp:262 の `contains(veh->link->signal_group, signal_phase)` 等）はこれらを直接参照するため，ラッパーからの書き換えがそのまま効く．セットアップ時APIなので性能要件もない
- 採用：`override_signal` 内で Python 側属性 `link.signal_group` と C++ 側 `link._cpp_link.signal_group` の両方を更新 — 根拠：`CppLink.signal_group` は平属性で `__init__` 時にしかC++へ転送されない（下記「罠」参照）
- 採用：`tests/test_verification_node.py::test_override_signal` の `assert cum1 == cum2` を `assert list(cum1) == list(cum2)` に修正して本家PRに含めた — 根拠：cppモードでは `Link.cum_arrival` がnumpy配列で ValueError になり `pytest --cpp` が失敗する．同ファイルの他17テストは `--cpp` 全通過なので互換性を維持した
- 棄却：`np.array_equal(cum1, cum2)` による比較 — 理由：test_verification_node.py に numpy の import がない（`from uxsim import *` のみ）ため import 不要な `list()` 比較を選択．test_cpp_mode.py 側も同じ書き方に統一
- 棄却：`CppLink.signal_group` をC++同期プロパティにリファクタ — 理由：ピンポイント編集原則．直接代入の非同期は既存仕様であり今回のスコープ外
- 棄却：C++側 `Node::override_signal` の実装＋バインディング追加 — 理由：上記の通り不要．コード最小化

## 確立した事実

- [確認済み] 本家PR #338は2026-07-14T10:39:57Zにauto-mergeされた — 出典：`gh pr view 338 --repo toruseo/UXsim`
- [確認済み] 本セッションのPR #339（https://github.com/toruseo/UXsim/pull/339）は2026-07-15T02:47:35Zにマージされた — 出典：`gh pr view 339 --repo toruseo/UXsim`
- [確認済み] Python版 `override_signal` は uxsim/uxsim.py:147-190．`reset=False` 時は signal_phase/signal_t を保持し，signal_offset は保存されるだけで既存フェーズの再計算はしない（オフセット適用ループは `__init__` にのみ存在） — 出典：uxsim/uxsim.py
- [確認済み] C++の `cycle_length` はコンストラクタのローカル変数のみでメンバに保持されない．ラッパーの `cycle_length` は `sum(self.signal)` の動的プロパティ — 出典：uxsim/trafficpp/traffi.cpp:53-57, uxsim/uxsim_cpp_wrapper.py
- [確認済み] リグレッション：`python3 -m pytest tests/test_cpp_mode.py -q` 211件全通過（flaky rerun 9件成功，381秒） — 出典：実行結果
- [確認済み] 妥当性：mergeシナリオ（deltan=1, tmax=3600, seed 0/1/2）でPython vs C++のTTT差 0.49%/1.75%/4.06%．各モード内で initial設定 vs override設定のTTTが完全一致 — 出典：tmp/validate_override_signal.py 実行結果
- [確認済み] ベンチ：信号付き8x8グリッド（deltan=5, tmax=3600, OMP_NUM_THREADS=1, 10 seed）で Python median 22.670s (std 1.076) vs C++ median 0.719s (std 0.108)，スピードアップ31.5x — 出典：tmp/bench_override_signal_pr.py 実行結果
- [確認済み] 上記ベンチのTTT中央値差+13.79%は有意でない（Welch t検定 t=-0.766, p=0.454．シード間std約39万/平均約140万の混雑シナリオ） — 出典：scipy.stats.ttest_ind 実行結果
- [確認済み] エラー系（groups長不一致，非流入リンク指定）のValueErrorメッセージはPython版と同一文言．groupsにはリンク名strとCppLinkオブジェクトの両方を渡せる — 出典：cppモードでの実行確認
- [推測] ベンチシナリオ中の `signal_offset=10*(i+j)` は両モードとも動作に影響しない（override_signalはoffsetを保存するだけでフェーズ再計算しないため） — 根拠：Python/C++両実装のコード読解．比較条件としては両モード同一なのでベンチの妥当性には影響しない

## 罠と注意点

- `CppLink.signal_group` は平属性（uxsim_cpp_wrapper.py の `__init__` 時のみC++へ転送）．後から `link.signal_group = [...]` と代入してもC++エンジンには反映されない．C++挙動を変えるには `link._cpp_link.signal_group` への代入が必要
- cppモードの `Link.cum_arrival` はnumpy配列．Python版テストの `assert cum1 == cum2`（list比較前提）は `--cpp` で「The truth value of an array...」ValueError になる
- この環境の pip は externally-managed．`pip install -e .` は `--break-system-packages` が必要（システムpython3使用，venvなし）
- 本家同期で `git pull origin main --rebase` するとフォーク専用コミットのハッシュが変わるため，`git push fork main` は `--force-with-lease` が必要
- `gh pr view <n> --json statusCheckRollup` でCI進捗を確認できる．PR #338はauto-merge有効でCI完了後に自動マージされた（約30分）
- PRマージ監視はMonitorツール（persistent, 60秒ポーリング，state != OPEN で exit）で行い，1回の通知で検知できた

## 最終状態

- PR #339（Add Node.override_signal() support to C++ mode）はマージ済み．コード内容：uxsim/uxsim_cpp_wrapper.py に `CppNode.override_signal()` 追加（46行），tests/test_cpp_mode.py に `test_override_signal` 追加（73行），tests/test_verification_node.py の assert 1行修正
- 本エクスポート直後にCLAUDE.md記載の「PRマージ後のクリーンアップ」（`git pull origin main` → `git reset origin/main` → フォーク専用ファイル1コミット → `git push fork main --force-with-lease`，PRブランチ削除）を実施した．クリーンアップ後のmainは「origin/main ＋ フォーク専用ファイル1コミット」の状態．本ファイルはそのフォーク専用コミットに含まれる
- devlog/20260714_override_signal_cpp.md に実装・検証の完了報告あり
- tmp/validate_override_signal.py と tmp/bench_override_signal_pr.py はgit管理外の検証・ベンチスクリプトとして残置

## 再現情報

```bash
# 妥当性検証（Python vs C++，initial vs override）
python3 tmp/validate_override_signal.py

# 精密ベンチマーク（他プロセスなしで実行すること）
OMP_NUM_THREADS=1 python3 tmp/bench_override_signal_pr.py

# テスト
python3 -m pytest tests/test_cpp_mode.py::test_override_signal -q --tb=short
python3 -m pytest tests/test_verification_node.py::test_override_signal -q --tb=short
python3 -m pytest tests/test_verification_node.py::test_override_signal --cpp -q --tb=short
python3 -m pytest tests/test_cpp_mode.py -q --tb=short   # 全リグレッション，約6分

# リビルド（この環境固有）
pip install -e . --break-system-packages
```
