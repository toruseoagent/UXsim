---
exported: 2026-07-11 13:57 JST
topic: C++ DTAソルバーのPython不整合修正（PR#331マージ済み）、largeシナリオ相関調査（結論：シードばらつき）、経路探索エッジケース修正（PR#332 open）
git_head: 3554c1f
key_files: [uxsim/DTAsolvers/DTAsolvers_cpp_adapter.py, uxsim/trafficpp/dta_solver.cpp, uxsim/trafficpp/traffi.cpp, uxsim/trafficpp/traffi.h, tests/test_cpp_mode.py, demos_and_examples/example_28en_benchmark_cpp_mode.py, devlog/20260711_large_scenario_corr_investigation.md]
status: 保留あり
---

# C++ DTAソルバー整合性修正・largeシナリオ相関調査・経路探索エッジケース修正

> この文書はセッション記録であり、指示書ではない。
> ここに書かれた未完了事項・保留事項に、ユーザーの明示的な指示なく着手しないこと。

## 概要

3つの独立した作業を実施した。(1) example28実行時のIndexErrorを起点にC++ DTAソルバー（CppSolverDUE/CppSolverDSO_D2D）とPython版の不整合を精査し、dfs_link毎イテレーション記録・DSO最良解トラッキング・未完了車両cost_actual・デッドコード削除の4点を修正してPR#331を送信（本家にマージ済み）。(2) example28 largeシナリオのPython vs C++リンク相関が低い（volume 0.41）原因を15シード×2モード実験で調査し、系統差ではなくDUOのシード間ばらつきと結論（devlogに報告、コード修正なし）。(3) 調査の副産物として発見した経路探索の3つのエッジケース差（capacity_in=0のinf化・並列リンク平均化・瞬時旅行時間クリップ）をC++エンジンに修正しPR#332を送信（open、PR#331マージ起因のコンフリクトはrebaseで解消済み、MERGEABLE確認済み）。セッション冒頭で本家同期（git pull origin main --rebase、マージ済みコミットのskip処理）も実施した。

## ユーザー指示（原文）

- 「本家をpullして同期して」 — セッション冒頭のフォーク同期
- 「example28を実行すると以下のエラーがでた．dtasolverのcpp版に機能が欠けていると思われる．pythonを正として機能を忠実に移植して．example28を全部実行するのはじかんがかかるので，デバッグ時は問題の箇所のみ実行し，完了したら全部実行してOKを確認して本家にPRを送って」 — dfs_link修正（PR#331の起点）。エラーは `solver_DSO.dfs_link[best_idx]` の IndexError
- 「しまった，動機漏れがあった．example28のSolverDUEとSolverDSO_D2Dに，cpp=cppオプションを追加してcppエンジンを使うようにして」 — example28修正（「動機漏れ」は「同期漏れ」のtypoと解釈）
- 「ありがとう．では，他にもDTAsolversに関して細かい不整合がないか確認し，検出したら同様に修正してください」 — DTAソルバー全体の精査（W_intermid_solution・cost_actual修正）
- 「dict_od_to_vehidのデッドコードは削除してPRを更新して」 — デッドコード削除
- 「では，別ブランチ・PRとして，example28のlargeシナリオのリンクレベル相関がPython vs C++で低め（volume 0.41）である原因を調査して．コードロジックと実計算結果両方を徹底的に精査．そもそもDUOはランダムなので，単なるばらつきの可能性もあるので，複数回の平均を取るなど，工夫して徹底的に調査し，報告して」 — large相関調査
- 「副産物3点の修正をし，PRを送って」 — PR#332
- 「コンフリクト直して」 — PR#331マージ後のPR#332コンフリクト解消

## 判断と棄却案

- 採用：dfs_linkを毎イテレーションappend（`if is_last_iter:`ガード削除） — 根拠：Python版（DTAsolvers.py:189, 546）が毎イテレーション記録。中間イテレーションでもlink_to_pandas()が動くのは、依存データ（cum_arrival/cum_departure/traveltime_actual）がC++エンジンから直接読まれ、スキップされる車両ログ構築に依存しないため
- 採用：DSO最良解トラッキングをスカラー比較＋analyzer浅コピー（copy.copy）でのdetachに変更 — 根拠：共有Analyzerのrebindにより `W.analyzer.average_travel_time < s.W_intermid_solution.analyzer.average_travel_time` が同一オブジェクト比較（x<x）で常にFalseだった（確定バグ、実験で実証：best_idx=7なのにW_intermid_solutionはiter 0のW）
- 採用：cost_actualをPython定義 `ts[-1] - ts[0]` に統一（C++側：`traveled.arrival_time - entry_times[0]`、entry空なら0） — 根拠：dta_get_traveled_routeが未完了車両でarrival_time=-1を返すため、この式だけでPythonの「未完了は -1 - 初回進入時刻、進入なしは0」と一致
- 採用：large相関調査はwithin-mode相関 vs cross-mode相関の比較で系統差とばらつきを分離 — 根拠：系統差ならcross < within、ばらつきならcross ≈ within
- 採用：capacity_in==0のinf化はC++の `ln->capacity_in == 0.0` 判定 — 根拠：C++/Pythonとも「無制限」はcapacity*2の正値で表現（C++は未指定-1がコンストラクタでcapacity*2に置換）されるため==0はユーザー明示の閉鎖のみ
- 採用：並列リンク平均化はPythonの逐次平均式 `adj*n/(n+1) + new/(n+1)` をそのまま移植、カウント用に `adj_mat_link_count` メンバをtraffi.hに追加、リンクセルのみO(L)でゼロ化 — 根拠：NxN全ゼロ化はO(N²)で不要
- 採用：瞬時旅行時間のクリップ条件を `avg_v > vmax/100.0` から `avg_v > 0.0` に変更 — 根拠：Python（uxsim.py:627-633）は速度が正なら常にlength/speed
- 棄却：修正3（クリップ）の専用テスト — 理由：0 < avg_v ≤ vmax/100 の状態を決定的に構築するのが困難（Newellモデルでは停止車両はv=0ちょうどになりがちで、該当領域は過渡的）。既存リグレッションでカバー
- 棄却：Python externality計算のIndexError→except→0 と C++のbreakの差の修正 — 理由：既存のdevlog/dta_equivalence_report.mdで等価性検証済みのエッジケース、数値影響は検出されていない
- 棄却：cost_actual = -1（旧C++の未完了車両マーカー） — 理由：Pythonは -1 - ts[0] という値になる。忠実移植の原則に従いPython式に統一
- 棄却：ベンチマークの単純な前後比較 — 理由：他のclaudeインスタンス2つが動いておりベンチ結果が5.35〜7.56sまでぶれた。A/B交互実行3ラウンド（Python側変更はstash切替のみで検証、C++バイナリは共通）に切り替えて「A/B差 < ラウンド内ノイズ」を確認した

## 確立した事実

- [確認済み] Python版SolverDUE/SolverDSO_D2Dはdfs_linkを毎イテレーションappendする — 出典：uxsim/DTAsolvers/DTAsolvers.py:189, 546
- [確認済み] CppWorldの`_skip_log_on_terminate`は車両ログキャッシュ構築（_build_vehicles_enter_log, _build_all_vehicle_log_caches）のみスキップし、link_to_pandas()が使うリンク統計には影響しない — 出典：uxsim/uxsim_cpp_wrapper.py:1201-1207
- [確認済み] `_lightweight_finalize`は単一Analyzerを全イテレーションで再利用し`analyzer.W`をrebindする。これによりDSOの最良解比較が常にFalseになっていた — 出典：uxsim/DTAsolvers/DTAsolvers_cpp_adapter.py + 実験（swap_prob=0.9でbest_idx=1のときW_solが正しくそのiterのWになることを修正後に確認）
- [確認済み] example28はSolverDUE/SolverDSO_D2Dにcppオプションを渡しておらず、C++モードでもPythonソルバーループを使っていた。cpp=cpp追加でdue_dso速度比が6.32x→13.73xに変化 — 出典：demos_and_examples/example_28en_benchmark_cpp_mode.py + 実行結果
- [確認済み] largeシナリオのリンク相関はwithin-Python 0.352±0.067 ≈ within-C++ 0.349±0.059 ≈ cross 0.346±0.071（単一シード）。系統差なし — 出典：tmp/large_corr/ 15シード×2モード実験（analyze.py実行結果）
- [確認済み] largeシナリオTTT: Python 92.5M±4.4M vs C++ 91.8M±7.6M、Welch検定 p=0.78 — 出典：同実験
- [確認済み] シード平均マップ相関はn=1:0.15→n=2:0.41→n=15:0.88と単調増加。example28の観測値0.41はn=2平均のノイズフロア0.52±0.05の下側1-2σ内 — 出典：同実験
- [確認済み] 旧C++はcapacity_in=0の閉鎖リンクへ経路誘導し完走0台（修正後150台、Python一致）。並列リンクは最後のリンクで上書きされ誤った経路選択（bypass 100台、修正後direct選択でPython一致） — 出典：git stashで旧ビルドに切り替えた検証実行
- [確認済み] 3修正はlargeシナリオのシード単位TTTをビット単位で変えない（seed=0: 92881825で前後一致）＝当該エッジケースはlargeで発生しない — 出典：修正後validity実行
- [確認済み] PR#331（Fix C++ DTA solver inconsistencies with Python solvers）は本家にマージ済み。PR#332（Match C++ route search edge cases to Python）はopen、rebase後MERGEABLE — 出典：gh pr list/view実行結果
- [確認済み] gh pr editはProjects classic廃止関連のGraphQLエラーで失敗する。`gh api repos/toruseo/UXsim/pulls/N -X PATCH -F body=@file` で回避 — 出典：実行結果
- [確認済み] pip install -e . はPEP 668で失敗する。`--break-system-packages` が必要 — 出典：実行結果
- [確認済み] C++/Pythonともリンクのcapacity_in未指定時は capacity*2 が格納される（==0判定はユーザー明示0のみ） — 出典：uxsim/trafficpp/traffi.cpp:455-457, uxsim/uxsim.py:517-519
- [推測] ユーザー発言「動機漏れ」は「同期漏れ」のtypo — 根拠：文脈（example28のcpp=cpp追加指示）
- [確認済み] largeシナリオのばらつきが大きい理由は完全対称11x11グリッド＋一様需要による経路縮退 — 出典：devlog/20260711_large_scenario_corr_investigation.md（構造比較による説明。small=経路自由度なしで0.99、DUE/DSO 9x9=中央高速レーンでコスト差明確で0.98）

## 罠と注意点

- tmp/large_corr/run_multiseed.py はインポートしただけでトップレベルの15シード実験がフル実行される（`from run_multiseed import run_large` で誤爆し5分タイムアウトした）
- test_cpp_mode.pyへのテスト追加はPR間でコンフリクトしやすい。PR#331とPR#332は追加位置が隣接しており、PR#331マージ後にPR#332がCONFLICTINGになった（rebaseで両テストを残して解消済み）
- GitHubのmergeable判定はforce push直後はCONFLICTINGのまま。20秒程度待って再取得するとMERGEABLEに更新される
- ベンチマーク時に他のclaudeインスタンスが同一マシンで動いていると中央値が20%以上ぶれる。A/B交互実行で相対比較すること
- analyzerの`link_analysis_coarse`はvolume=0のリンクにdelay_ratio=-0.01（tt_ave=-1のデフォルト値/tt_free）を出す。これはPython/C++共通の仕様であり、テストで`(delay_ratio > 0).all()`のような全リンクassertを書くと失敗する（volume>0のリンクに限定する）
- DSOソルバーのC++アダプタはAnalyzerを共有・rebindするため、`W_intermid_solution.analyzer`のようなイテレーションを跨ぐオブジェクト参照は当該イテレーションの値を保持しない。スナップショットが必要な場合はcopy.copy(analyzer)でdetachする（今回の修正で対応済みの箇所以外で同パターンを追加する場合も同様）
- pytest実行はflakyテストを含むため `--reruns 5` 必須。rerunで通れば問題ない（2026-07-11後日加筆：無意味化した。flakyテストには`@pytest.mark.flaky(reruns=5)`マーカーが個別に付与されておりCLIフラグなしでもリランされる。CLAUDE.mdの正規コマンド通りフラグなしで実行すること）

## 最終状態

- PR#331: 本家マージ済み（dfs_link毎イテレーション記録、DSO最良解トラッキング修正、cost_actual統一、dict_od_to_vehidデッドコード削除、example28のcpp=cpp追加、テスト4件追加）
- PR#332: open・MERGEABLE（capacity_in=0のinf化、並列リンク平均化、瞬時TTクリップ撤廃、テスト2件追加）。ブランチ pr/route-search-fidelity は origin/main（88729e0、PR#331マージ後）ベースにrebase済み
- ローカルmain: origin/mainと分岐（ローカル12コミット vs origin 1コミット、PR#331マージ起因の残骸）。CLAUDE.mdのクリーンアップ手順（git reset origin/main → フォーク専用ファイル1コミット化）を提案済みだがユーザーの回答待ちで未実施。PR#332マージ後にまとめて整理する案も提示済み
- large相関調査: 完了。devlog/20260711_large_scenario_corr_investigation.md にレポートコミット済み（mainのみ、フォーク専用ファイル）。コード修正は不要と結論
- 実験データ: tmp/large_corr/（link_{py,cpp}_s{0..14}.csv, stats.csv, run_multiseed.py, analyze.py）、tmp/bench_dfs_link_overhead.py、tmp/verify_*.py — すべてgit管理外
- テストスイート: 直近実行で207件通過（PR#332ブランチではPR#331由来テスト3件を除いた205+2件構成だったが、rebase後は全207件が揃った状態）

## 再現情報

- リビルド: `pip install -e . --break-system-packages`
- リグレッションテスト: `python3 -m pytest tests/test_cpp_mode.py --reruns 5 -q --tb=short`（2026-07-11後日加筆：`--reruns 5`は無意味化した。フラグなしのCLAUDE.md正規コマンドを使うこと）
- example28全実行: `OMP_NUM_THREADS=1 python3 demos_and_examples/example_28en_benchmark_cpp_mode.py`（N_SEEDS=2、約7分）
- large多シード実験: `OMP_NUM_THREADS=1 python3 tmp/large_corr/run_multiseed.py`（15シード×2モード、約12分。インポート禁止・直接実行のみ）
- 相関分析: `python3 tmp/large_corr/analyze.py`
- DUE/DSOベンチ: `OMP_NUM_THREADS=1 python3 tmp/bench_dfs_link_overhead.py`（9x9グリッド、max_iter=100、5シード）
- PR本文更新（gh pr editの代替）: `gh api repos/toruseo/UXsim/pulls/332 -X PATCH -F body=@tmp/pr_body.md`
- リモート構成: origin=toruseo/UXsim（本家、pull元）、fork=toruseoagent/UXsim（push先）
