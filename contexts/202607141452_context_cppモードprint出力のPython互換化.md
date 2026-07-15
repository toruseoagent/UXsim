---
exported: 2026-07-14 14:52 JST
topic: C++モードのシミュレーション中print出力（シナリオ情報・進捗表示）をPython版と同一仕様にし，PR #337を作成
git_head: 7b41f18
key_files: [uxsim/trafficpp/traffi.h, uxsim/trafficpp/traffi.cpp, uxsim/trafficpp/bindings.cpp, uxsim/uxsim_cpp_wrapper.py, tmp/valid_bench_print_parity.py]
status: 完了
---

# C++モードprint出力のPython互換化とPR #337作成

> この文書はセッション記録であり、指示書ではない。
> ここに書かれた未完了事項・保留事項に、ユーザーの明示的な指示なく着手しないこと。

## 概要

`World(cpp=True)` のシミュレーション時print出力をPython版と同一仕様にした．従来C++モードはシナリオ情報（"simulation setting:" ブロック）を出力せず，進捗表示も独自形式（"Simulating..." ヘッダ，total_timesteps/10間隔，computation time列なし）だった．example_01en_basicの全標準出力をPython/C++両モードでdiff比較し，数値以外完全一致を達成．リグレッションテスト210件通過，妥当性検証（TTT差3.2%以内），ベンチマーク（25.9倍）を実施し，PR https://github.com/toruseo/UXsim/pull/337 を作成した．

## ユーザー指示（原文）

- 「C++モードでシミュレーション中にprintする内容をpythonと完全に同じ仕様にして．シミュレーション直前のシナリオ情報，シミュレーション中の途中経過（特に経過時間）が違っていると思う．example_01en_basicを対象に，数値以外は全く同じprintが観測されるようにして」 — 本体実装
- 「PR送って」 — PR作成（CLAUDE.mdのPR前必須チェック実施の上）

## 判断と棄却案

- 採用：シナリオ情報と "simulating..." はラッパー（`uxsim_cpp_wrapper.py` の `finalize_scenario`）でPython側からprint — 根拠：Python版 `finalize_scenario` と同じ動線（stats → `sim_start_time = time.time()` → "simulating..."）を最短で再現できる
- 採用：シミュレーション中の進捗行はC++の `main_loop` 内から出力 — 根拠：main_loopは全ステップをC++内で回すため途中経過はC++からしか出せない
- 棄却：wrapperの `exec_simulation` を `show_progress_deltat` 間隔で区切って `main_loop` を複数回呼び，進捗をPython側でprintする案 — 理由：C++側で出力する方が構造が単純で，main_loop分割はPythonオーバーヘッドと挙動差リスクを増やす（検討のみ，未実装）
- 採用：進捗行の車両数・平均速度は「リンク上の全platoonの数×delta_n」と「全platoonのvの単純平均」で計算 — 根拠：Python版 `Analyzer.show_simulation_progress` の Σ(density×length) と Σ(density×length×speed)/Σ(density×length) は数学的にこれと等価（リンクごとのdensity=num_vehicles/length，speed=リンク上車両のv平均のため，重み付き平均が全車両平均に帰着する）
- 採用：計測時間はC++に `std::chrono::steady_clock` ベースの `sim_start_time` を持たせ，wrapperの `finalize_scenario` でPython側 `time.time()` 記録と同時に `reset_sim_start_time()` を呼ぶ — 根拠：Python版はfinalize時点からの経過秒を表示するため，開始時刻の同期が必要
- 採用：進捗行フォーマットはC++側で `snprintf("%8.0f s| %8.0f vehs|  %4.1f m/s| %8.2f s", ...)` — 根拠：Pythonのf-string `{TIME:>8.0f} s| {sum_vehs:>8.0f} vehs|  {avev:>4.1f} m/s| {elapsed:8.2f} s` とprintfフォーマットが一対一対応する
- 採用：`CppLink.__init__` で `self.length = length`（ユーザー渡し値を保持，C++読み戻しをやめる）— 根拠：C++はlengthを無加工で保持（traffi.cpp Link ctor で `length(length)`）しており値は同一．Python版はintで渡せばintのまま表示（"3000 m"）するが，C++読み戻しではfloat化して "3000.0 m" になっていた
- 採用：`_build_all_vehicle_log_caches` で `DELTAT` がintのとき `log_t` をint64にキャスト — 根拠：Python版のlog_tはT*DELTAT（DELTATがintならint）で，`vehicles_to_pandas` のt列が `15` vs `15.0` と表示され列幅も変わっていた
- 採用：`exec_simulation` を終了後に再呼び出しした場合，wrapperで `analyzer.show_simulation_progress()` を呼んでからterminated — 根拠：Python版の `start_ts == end_ts == TSIZE` 分岐と同一挙動にするため（analyzerはPython共通実装で，CppLinkのdensity/speedプロパティ経由で動作する）
- 採用：t=0のヘッダ+初期行は `print_mode` のみで出力（`show_progress` 不問）— 根拠：Python版 `exec_simulation` のT==0分岐が `W.print`（print_modeのみゲート）であるため

## 確立した事実

- [確認済み] C++の `writer` は bindings.cpp の `create_world()` で Python `sys.stdout` にリダイレクトされる（`py_stdout_redirect_buf` が `PyObject_CallMethod(py_stdout_raw, "write", ...)` を呼ぶ）ため，Python printとC++出力の順序は保たれ，flush対策は不要 — 出典：uxsim/trafficpp/bindings.cpp 90-181行付近の実行前調査
- [確認済み] Python版の進捗出力タイミングは3箇所：ループ内T==0でヘッダ+初期行（print_modeのみゲート），ループ内 `T%show_progress_deltat_timestep==0 && T>0`（print_mode && show_progress），exec_simulation末尾 `W.T==W.TSIZE` 時の最終行（TIME=TMAX）— 出典：uxsim/uxsim.py exec_simulation（2386-2440行付近）
- [確認済み] Python版の進捗行フォーマットは `f"{s.W.TIME:>8.0f} s| {sum_vehs:>8.0f} vehs|  {avev:>4.1f} m/s| {time.time()-s.W.sim_start_time:8.2f} s"` — 出典：uxsim/analyzer.py `show_simulation_progress`（1150-1164行付近）
- [確認済み] example_01en_basicの出力はPython/C++で数値以外完全一致（シナリオ情報・進捗・analyzer pandas出力すべて）．残る差は setup/computation time，tqdm処理速度，乱数由来のシミュレーション値のみ — 出典：scratchpadでの `diff ex01_out_py.txt ex01_out_cpp.txt` 実行結果
- [確認済み] エッジケース（`duration_t=100` 分割実行，終了後の再 `exec_simulation()`，`show_progress=0`，`print_mode=0`，`until_t=700` 部分実行）も両モードで数値以外一致 — 出典：scratchpad cmp_edge.py の diff 実行結果
- [確認済み] `tests/test_cpp_mode.py` 210件通過（6件flaky rerunで成功，396.80s）— 出典：pytest実行結果
- [確認済み] 妥当性検証（11x11グリッド，tmax=7200，deltan=3，3seed）：TTT差 seed42=3.20%/seed1=2.60%/seed2=1.20%，完了トリップ数差0.00%（29040で一致），平均速度差0.42〜2.91% — 出典：tmp/valid_bench_print_parity.py 実行結果
- [確認済み] ベンチマーク（同シナリオ，OMP_NUM_THREADS=1，print_mode=0）：C++全体中央値0.956s，Python全体中央値24.789s，スピードアップ25.9倍 — 出典：同上
- [確認済み] PR #337 を toruseoagent:pr/print-parity → toruseo/UXsim に作成済み — 出典：gh pr create実行結果 https://github.com/toruseo/UXsim/pull/337
- [確認済み] 旧C++進捗表示で使っていた main_loop 内の `veh_count`/`speed_sum` 累積は削除した（print専用だったため）— 出典：uxsim/trafficpp/traffi.cpp の編集
- [確認済み] `pip install -e .` はこの環境ではPEP 668で拒否されることがあり `--break-system-packages` が必要 — 出典：ビルド実行結果
- [推測] 進捗行の平均速度の小さな数値差（例：600s時点で17.5 vs 17.2 m/s）は乱数非互換（マージ優先順位の抽選）由来 — 根拠：CLAUDE.mdに乱数生成は完全互換ではないと記載があり，merge_priority指定ありのexample_01en本体では17.5で一致した
- [未確認] `show_progress_deltat < DELTAT` のとき（`show_progress_deltat_timestep`が0）Python版は `W.T % 0` でZeroDivisionErrorになるはず — C++側は `> 0` ガードで出力なしにした．Python側の実挙動は未検証

## 罠と注意点

- `sleep 60` を含むコマンドはこの環境のハーネスにブロックされる．長時間待ちはバックグラウンド実行（run_in_background）の完了通知か Monitor を使う
- C++ `main_loop` のループ後，メンバ `time` は `(end_ts-1)*delta_t` のまま残る．最終進捗行は `timestep * delta_t` をローカル計算して渡した（メンバ `time` の更新はしていない）
- `vehicles_to_pandas` のt列dtypeはwrapperの `_build_all_vehicle_log_caches` で決まる．home区間の `home_t` 生成dtype（`np.arange(nm, dtype=all_log_t.dtype)`）も揃えないとconcatenateでfloatに昇格する
- ベンチのC++ seed=1でstd=1.139と大きい外れ値が出た（初回実行のウォームアップと思われるが未確認）．中央値は安定（0.913〜0.991s）
- CLAUDE.mdの規定：ベンチ実行中は他プロセスを走らせない．PRブランチはorigin/mainベースでcherry-pick．フォーク専用ファイル変更はコードコミットに同梱しない（今回フォーク専用ファイルの変更はなし）

## 最終状態

- mainブランチ：origin/main + フォーク専用コミット(476bcf9) + コードコミット(7b41f18) をfork（toruseoagent/UXsim）にpush済み
- pr/print-parityブランチ：origin/mainベース + cherry-pick(bf8929b) をforkにpush済み，PR #337 open
- 変更ファイル：uxsim/trafficpp/traffi.h（show_progress/show_progress_deltat_timestep/sim_start_timeフィールド，reset_sim_start_time/print_progress_line宣言），traffi.cpp（コンストラクタ初期化，print_progress_line実装，main_loop進捗出力書き換え），bindings.cpp（def_rw 2件+reset_sim_start_time），uxsim_cpp_wrapper.py（finalize_scenarioでのstats/simulating出力とC++設定伝達，exec_simulation終了後分岐の進捗行，CppLink.lengthのユーザー値保持，log_tのint64キャスト）
- tmp/valid_bench_print_parity.py：妥当性検証+ベンチスクリプト（git管理外，コミットしていない）
- .claude/ と tmp/ は未追跡のまま（コミット対象外）
- PRマージ後のmainクリーンアップ（CLAUDE.md「フォーク同期の運用」）は未実施（PRがopenのため）

## 再現情報

- 出力比較（example_01en_basic）：
  ```bash
  cd /tmp/claude-1000/-home-toruseo-UXsim-dev-cpp/a0e7f36f-59cb-4e0b-851f-b2b005b42a8d/scratchpad
  cp /home/toruseo/UXsim-dev-cpp/demos_and_examples/example_01en_basic.py ex01_py.py
  sed 's/^W = World($/W = World(\n    cpp=True,/' ex01_py.py > ex01_cpp.py
  python3 ex01_py.py > ex01_out_py.txt 2>&1; python3 ex01_cpp.py > ex01_out_cpp.txt 2>&1
  diff ex01_out_py.txt ex01_out_cpp.txt
  ```
  （scratchpadはセッション固有のため消えている可能性あり．スクリプト内容は上記コマンドで再生成可能）
- リビルド+テスト：`pip install -e . --break-system-packages && python3 -m pytest tests/test_cpp_mode.py -q --tb=short`
- 妥当性検証+ベンチ：`OMP_NUM_THREADS=1 python3 tmp/valid_bench_print_parity.py`（リポジトリに現存）
- PR作成：`git checkout -b pr/print-parity origin/main && git cherry-pick 7b41f18 && git push fork pr/print-parity && gh pr create --repo toruseo/UXsim --head toruseoagent:pr/print-parity --title "Match C++ mode print output to Python during simulation" --body "..."`
