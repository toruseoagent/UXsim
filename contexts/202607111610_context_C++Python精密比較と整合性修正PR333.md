---
exported: 2026-07-11 16:10 JST
topic: uxsim.pyとC++エンジンの精密比較（19差異検出）、うち12件を修正しPR#333として本家にマージ、コメントのハードラップ一掃、マージ後クリーンアップ
git_head: 20c4d5c
key_files: [uxsim/trafficpp/traffi.h, uxsim/trafficpp/traffi.cpp, uxsim/trafficpp/bindings.cpp, uxsim/uxsim_cpp_wrapper.py, uxsim/DTAsolvers/DTAsolvers_cpp_adapter.py, tests/test_cpp_mode.py, devlog/20260711_cpp_python_precise_comparison_and_fixes.md]
status: 完了
---

# C++エンジンとPythonコアの精密比較・整合性修正（PR#333）・コメント規約整備

> この文書はセッション記録であり、指示書ではない。
> ここに書かれた未完了事項・保留事項に、ユーザーの明示的な指示なく着手しないこと。

## 概要

uxsim.py（コアシミュレータ）とC++エンジンを1行対応で精密比較し、計算実験で確認した19の差異を重要度別に報告した。ユーザー指示により13（hard_deterministicのタイブレーク）・14（heterogeneous_DUO未実装）・軽微5件を除く12件を修正し、1・2・6（自動TMAX・7200秒プレースホルダ・部分実行タイミング）にはPython/C++一致テストを追加した。修正コミット後に自作コメントの変更経緯ナラティブ除去、続いて既存コメントのハードラップ51箇所の再フロー（文言不変）を実施。妥当性検証（TTT平均差0.09%）と精密ベンチ（15.2倍）を経てPR#333を本家に送付、マージされ（fa74485）、mainのクリーンアップ（origin/main＋フォーク専用1コミット=20c4d5c）まで完了した。

## ユーザー指示（原文）

- 「pythonとC++のコードの差異を徹底的に確認して．対象はコアシミュレータ（uxsim.py）．エッジケースも含めて確認すること．コード精査後，実計算をしないと判断がつかない箇所は計算実験により確かめる」 — 調査フェーズの仕様
- 「13, 14，軽微**以外**を全て修正して．1, 2, 6は一致テストも追加」 — 修正スコープの確定（報告した差異番号を参照）
- 「claude.mdの規約にしたがい，ハードラップコメントを全て修正してください．また，今回の修正の文脈でのコメントが多く追加されたように見えます．文脈を共有していない第三者が読んでもわかるように書いてください」 — 自作コメントの是正
- 「では，既存のコメントのハードラップも修正してください．文章自体は変えないでください」 — 既存コメントの再フロー（文言変更禁止）
- 「では，本家にPRを送ってください」
- （このほか，--reruns 5の扱いをめぐり過去コンテキストの誤った採用問題が発生．経緯の原文引用は省略）
- 「その部分に後日加筆として無意味化したと書いておいて」 — 過去記録の--reruns 5記載への注記
- 「マージされたので，クリーンアップして」

## 判断と棄却案

- 採用：TMAX確定後のC++側リサイズは `World::set_t_max()` 新設（total_timesteps更新＋リンクのarrival_curve/departure_curve/traveltime_real/traveltime_instantをresize）で対応 — 根拠：C++ Worldは最初のaddNode時に生成され配列長が固定されるため、生成の遅延は不可能（ノード・リンク・需要が逐次C++に登録される構造）
- 採用：自動TMAXは `CppWorld._compute_auto_tmax()`（全車両の departure_time*DELTAT の最大から `(t//1800+2)*1800`）でPython finalize_scenarioと同式に — 根拠：旧実装の `_max_demand_t`（需要のt_end基準）はt_endが1800の倍数ちょうどのとき5400 vs 3600の差を生む
- 採用：exec_simulationはPythonと同一のend_ts計算（until_t/duration_t/duration_t2の3分岐＋TSIZEクランプ＋非正duration例外）をラッパー側で行い、C++ main_loopへは `until_t = end_ts*DELTAT` を渡す。TIMEは `T*DELTAT` — 根拠：旧変換（TIME+duration_t+DELTAT等）はチャンクごとに1ステップ超過
- 採用：add_demandはPythonの `range(int(t_start/Δ), int(t_end/Δ))` と同じ整数タイムステップループにし、出発時刻を `ts*Δ` に量子化 — 根拠：浮動小数点ループはDELTAT非整数倍境界で反復回数±1・出発時刻ceil化の差を生む（実験で5台vs6台、9台vs10台を確認）
- 採用：カスタム車両名対応のため `_sync_from_cpp` の「Python名=str(index)」仮定を廃し、`CppWorld._veh_by_index`（C++インデックス順のプロキシリスト）を導入。`_build_all_vehicle_log_caches`・`_build_vehicles_enter_log` も同リストを使用 — 根拠：VEHICLES辞書のキーが連番でなくなるとインデックス対応が壊れる。rename（pop+再挿入）は常に直近作成の末尾要素にのみ行うので挿入順=インデックス順は保たれる
- 採用：C++からの例外送出は std::invalid_argument（→ValueError）・std::out_of_range（→IndexError）を使いPythonと同じ例外型に — 根拠：nanobindの標準例外変換
- 採用：既存テスト test_coverage_addVehicle_taxi_and_links_avoid を `pytest.raises(ValueError)` 期待に変更 — 根拠：このテストは旧C++の「links_avoid全遮断を無視して走行」挙動（今回修正した差異10そのもの）を前提としており、Python版は同シナリオでValueErrorを送出することを実測確認した
- 採用：既存コメントの再フローは6ファイルの複数行コメントブロック・複数行docstringを機械抽出→全件目視判定→51件のold/newリテラル置換スクリプト（各置換の出現回数をassert）で適用 — 根拠：コメントアウトされたコード・ASCIIアート・@param列挙・信号現示リストを巻き込まない安全性
- 棄却：修正13（hard_deterministic_modeの最短経路タイブレーク差、scipy vs 自作Dijkstra）と修正14（heterogeneous_DUO）と軽微5件 — 理由：ユーザーが明示的にスコープ外と指定。13は両方とも正しい最短経路で原理的に完全一致は保証不能
- 棄却：bindings.cppのnanobind docstring（C文字列リテラル連結）のハードラップ扱い — 理由：コメントではなく式内部の文字列結合（実行時は1行の文字列）
- 棄却：Python本体の既存ラップ済みコメント（uxsim.py等）への波及 — 理由：編集対象外ファイル、ピンポイント編集の原則

## 確立した事実

- [確認済み] 検出した差異は19件（重大6・中程度8・軽微5）。修正した12件の詳細と対応は devlog/20260711_cpp_python_precise_comparison_and_fixes.md の表を参照 — 出典：tmp/precise_compare/experiments.py の実験結果と両実装のコード読解
- [確認済み] 修正前のC++は、tmax未指定＋需要が7200秒超のとき timestep=1440 で停止し exec_simulation()が0を返し続け check_simulation_ongoing()が永遠にTrue（完走9/60台） — 出典：exp3b実行結果
- [確認済み] 修正前は instantaneous_TT_timestep_interval がC++に未伝達（bindings def_rwはあるがwrapperが未設定、常に5）。interval=1指定時のtraveltime_instant変化点がC++はmod5=0のみだった — 出典：exp4実験
- [確認済み] 修正前は duo_update_time < DELTAT でC++の経路更新が一切走らず（timestep_for_route_update=0のガード）、短経路利用が24/60台（Python 60/60） — 出典：exp5実験
- [確認済み] 修正後、全実験項目（出発量子化・需要台数・T/TIME・自動TMAX・7200超完走・interval変化点・duo小時間・例外・信号/容量途中変更・log_s・average_speed）がPythonと一致。決定的シナリオでは累積曲線ビット単位一致 — 出典：tmp/precise_compare/experiments.py 修正後再実行
- [確認済み] 妥当性検証：9x9グリッド・288リンク・10,530台・10シードでTTT平均差0.09%（シード別最大0.82%）、完走数全シード一致。ベンチ：OMP_NUM_THREADS=1・10シードでPython中央値2.079s vs C++ 0.137s＝15.2倍 — 出典：tmp/precise_compare/validity_bench.py 実行結果
- [確認済み] tests/test_cpp_mode.py は210件通過（--reruns 5付きで実行。3 rerun等はflakyマーカー分を含む） — 出典：pytest実行結果
- [確認済み] PR#333「Match C++ engine behavior to Python core simulator」は本家にマージ済み（origin/main fa74485）。ローカルmainは fa74485＋フォーク専用1コミット（20c4d5c）に整理済み、PRブランチ pr/cpp-python-consistency はローカル・フォークとも削除済み — 出典：git log / gh pr view実行結果
- [確認済み] `--reruns 5` は不要。テストファイルに85箇所の `@pytest.mark.flaky(reruns=5)` マーカーがあり、CLAUDE.mdの正規コマンドはフラグなし。グローバルフラグはマーカーなしテストの真の失敗を隠しうる — 出典：grep結果とCLAUDE.md:193
- [確認済み] Pythonの `Vehicle.departure_time_in_second` は秒指定addVehicleで `departure_time*DELTAT` という既知バグ値を返す（uxsim.py:993のTODO）。C++は正値を返すため両モードで値が異なる（修正対象外の軽微差異として残存） — 出典：exp1実行結果
- [推測] exp12等でTTTが両モード完全一致するのは、需要時刻がDELTATの倍数で経路自由度がないシナリオでは実効的な乱数依存がないため — 根拠：累積曲線のビット単位一致

## 罠と注意点

- tests/test_cpp_mode.py は冒頭で `from numpy import *` しており `sum` がnp.sumになる。`sum(1 for ...)`（ジェネレータ）はTypeErrorで落ちる。リスト内包を使うこと
- 一致テストでuntil_tを段階指定するとき、前チャンクの到達タイムステップより過去のuntil_tを与えるとPython側が正しく例外を出す（duration非正エラー）。テスト設計時は単調増加のuntil_tにする
- PRマージ後の `git branch -d` はマージでSHAが変わるため「not fully merged」で拒否される。内容がマージ済みであることを確認して `-D` を使う
- CppNode.signal のgetterはC++からのコピーを返すため、`node.signal[0] = 10` のようなin-place変更はエンジンに反映されない（再代入 `node.signal = [...]` は反映される）。Pythonモードはlist共有のためin-placeも効く（残存する微小差異）
- 質問（？付き）にはまず回答そのものを完結させる。回答を後回しに検証・作業を始めるとユーザーに中断される（本セッションで実際に発生）。ユーザーのグローバルCLAUDE.mdにも「ユーザーからの問いかけにはまず返事する」が追記された
- バックグラウンドでの `pytest | tail` は完了までoutputファイルが空のまま（tailのバッファリング）。TaskOutputのblock=trueで完了を待つのが確実
- ベンチ実行中は他プロセスを走らせない（過去セッションからの継続注意。今回はベンチを同期実行し他作業を止めた）

## 最終状態

- 本家 toruseo/UXsim main = fa74485（PR#333マージ済み）。フォーク toruseoagent/UXsim main = ローカルmain = fa74485＋20c4d5c（フォーク専用: CLAUDE.md, devlog/, contexts/）。コード差分ゼロ
- devlog/20260711_cpp_python_precise_comparison_and_fixes.md に差異一覧表と修正内容の完了報告をコミット済み
- contexts/202607111357_context_DTAソルバー整合性修正とlarge相関調査.md の--reruns 5記載2箇所に「無意味化した」後日加筆済み
- 修正対象外として残存する既知差異：hard_deterministicのタイブレーク、heterogeneous_DUO（無言でhomogeneousになる）、軽微5件（Node.number_of_lanesのNone/-1表現、C++内部統計のabort計上、departure_time_in_secondのPython側バグ値、自己ループリンクのroute_next[k][k]、vehicles_enter_log同時刻値）、signalのin-place変更
- メモリ: feedback_question_answer_only.md（？付き質問にはまず回答を完結させる）を作成・修正済み
- 実験・ベンチのアーティファクト: tmp/precise_compare/{experiments.py, validity_bench.py}（git管理外）

## 再現情報

- リビルド: `pip install -e . --break-system-packages`
- リグレッションテスト（正規コマンド）: `python3 -m pytest tests/test_cpp_mode.py -q --tb=short`
- 差異検証実験（全12項目、修正後は全一致）: `python3 tmp/precise_compare/experiments.py [exp名...]`
- 妥当性検証＋精密ベンチ: `OMP_NUM_THREADS=1 python3 tmp/precise_compare/validity_bench.py`（9x9グリッド、10シード×2モード、約30秒）
- 新規一致テストのみ: `python3 -m pytest tests/test_cpp_mode.py -q -k "auto_tmax or partial_run"`
- PR作成: `git checkout -b pr/<name> origin/main && git cherry-pick <code-commit> && git push fork pr/<name> && gh pr create --repo toruseo/UXsim --base main --head toruseoagent:pr/<name> --title "..." --body-file tmp/pr_body.md`
- マージ後クリーンアップ: `git fetch origin && git reset origin/main && git add CLAUDE.md devlog/ contexts/ && git commit && git push fork main --force-with-lease`
- リモート構成: origin=toruseo/UXsim（本家、pull元）、fork=toruseoagent/UXsim（push先）
