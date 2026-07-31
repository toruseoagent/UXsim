# UXsim C++エンジンプロジェクト

## プロジェクト概要

UXsimはPython製の交通流シミュレータ．このリポジトリでは，C++製の高速シミュレーションエンジンをオプションとして組み込んでいる．

- `W = World(cpp=True)` でC++エンジンに切り替え．Pythonとほぼ完全な互換性
- 互換性の例外：taxiモード・Pythonコールバック関数（user_function）・pickle/copy/save_scenario/load_scenario．厳密には浮動小数点精度・乱数生成も完全互換ではないが，通常の使用であれば問題ないので気にする必要はない

初期統合フェーズ（フェーズ1〜5）は完了済み．詳細は `devlog/CLAUDE_uxsimpp_integration_phase.md` および他のファイルを参照．

## 現在のタスク：Python新機能のC++追従

UXsim本体（Python版）に新機能が追加されるたびに，それをC++エンジンに反映する．

### 作業フロー

1. **uxsim.pyの変更を確認** — Python版の新機能・変更点を把握
2. **C++に移植** — `uxsim/trafficpp/traffi.h`, `traffi.cpp` に実装
3. **バインディング追加** — `uxsim/trafficpp/bindings.cpp` にnanobindバインディング
4. **ラッパー対応** — `uxsim/uxsim_cpp_wrapper.py` でPython側インターフェース
5. **テスト追加** — `tests/test_cpp_mode.py` にテスト追加（後述）
6. **リグレッションチェック** — Python/C++両モードでテスト通過を確認

### 編集対象ファイル

| ファイル | 役割 |
|---------|------|
| `uxsim/trafficpp/traffi.h` | C++エンジンヘッダ |
| `uxsim/trafficpp/traffi.cpp` | C++エンジン実装 |
| `uxsim/trafficpp/bindings.cpp` | nanobindバインディング（traffi.cppを直接インクルードするsingle translation unit） |
| `uxsim/uxsim_cpp_wrapper.py` | C++とPythonの薄い橋渡しラッパー |
| `tests/test_cpp_mode.py` | C++モード専用テスト |

**編集しないファイル**: `uxsim/uxsim.py`, `uxsim/analyzer.py`, `uxsim/Utilities.py`, `uxsim/DTAsolvers.py` — これらはPython本体．C++側から使えるようにする

## アーキテクチャ

- **C++内部実装**: ほぼ全機能はC++エンジン内部に実装．ラッパーはC++とPythonの薄い橋渡しのみ
- **World.__new__によるディスパッチ**: `cpp=True`時にCppWorldを返す．ファクトリ関数に置き換えるのはNG（isinstance等が壊れる）
- **analyzerとの互換性はデータ構造で担保**: CppWorldが同名・同型のコンテナ（ADJ_MAT, TSIZE, Q_AREA等）を用意
- **UXsimpp-main/**: 元のUXsimppリポジトリ内容（参考用，直接は使わない）

## テスト

### テスト構成

`tests/test_cpp_mode.py` に全C++モードテストが集約（171テスト）：

- **インラインテスト（145件）**: Python版テストからコピーし `World(cpp=True, ...)` を直接指定
- **Notebookデモテスト（4件）**: パッチセル注入方式で実行（01en/jp, 09en_DTA, 10en_signal）
- **Exampleスクリプトテスト（22件）**: sed的にcpp=True注入してsubprocess実行

### テスト実行コマンド

```bash
# C++モードテスト（test_cpp_mode.py）
pip install -e . && python3 -m pytest tests/test_cpp_mode.py -q --tb=short

# 全テスト（Python + C++）
python3 -m pytest tests/ -q --tb=short

# --cppフラグで既存テストをC++モードで実行（conftest.pyで実装）
python3 -m pytest tests/ --cpp -q --tb=short
```

### Pythonモードテストの実行方針

- Pythonモードの全テスト（`tests/` 全体）はローカルでは時間がかかるため，**pushしてGitHub Actionsに任せる**
- `git push fork main` 後に `gh run list --repo toruseoagent/UXsim` でCI結果を確認
- ローカルではC++モードテスト（`test_cpp_mode.py`）と主要なPythonテスト（`test_other_functions.py`等）のみ実行

### 新機能追加時のテスト追加方法

1. Python版テストが `tests/test_*.py` に追加されたら，そのテストコードを `tests/test_cpp_mode.py` にコピー
2. `World(` を `World(cpp=True, ` に置換
3. cppモード対象外の機能（pickle等）を使うテストは除外
4. DRLなど30分以上かかるテストも対象外

## フォルダ運用

- `devlog/` には**Markdownの計画書・完了報告のみ**を置く（コミット対象）
- `contexts/` にはセッション記録（/export-context2の出力）を置く（コミット対象，フォーク専用）
- 一時スクリプト・ベンチ結果・プロファイル等のアーティファクトはリポジトリ直下の `tmp/` に置く（**git管理外．コミット禁止**）

## Git運用

- このリポジトリは `toruseo/UXsim`（本家）をフォークした `toruseoagent/UXsim` の作業リポジトリ
  - リモート `origin`: `toruseo/UXsim`（本家，読み取り専用．新機能の取り込み元）
  - リモート `fork`: `toruseoagent/UXsim`（フォーク，push先）
  - push時は `git push fork <branch>`
  - 本家の最新を取り込むには `git pull origin main`
  - PRは `toruseoagent/UXsim` → `toruseo/UXsim` へ `gh pr create` で送る

## ブランチ運用

- **開発はmainブランチで行う**．随時 `git push fork main` でフォークにpush
- **本家へのPR時**：mainから `pr/<feature>` ブランチを作成し，フォーク専用ファイル（`CLAUDE.md`, `devlog/`, `contexts/`, `user_testing.ipynb`）を除外してコミット → `git push fork pr/<feature>` → `gh pr create --repo toruseo/UXsim`
- PRブランチはorigin/mainベースで作り，mainの変更をcherry-pickしてフォーク専用ファイルをreset/除外する

### PR送信前の必須チェック

PRを送信・更新する前に，以下を**必ず**実行すること（省略不可）：

1. **リグレッションテスト**: `python3 -m pytest tests/test_cpp_mode.py -q --tb=short` — 全テスト通過を確認
2. **妥当性検証**: Python版とC++版で同一シナリオを実行し，主要な出力指標（TTT等）が数%以内で一致することを確認．十分な規模・反復回数で検証すること
3. **精密ベンチマーク**: 1スレッド（OMP_NUM_THREADS=1）で複数seed計測．中央値+stdを記録．スピードアップ倍率をPR本文に記載．**ベンチ実行中は他のプロセスを走らせない（CPUを占有して計測がぶれるため）**

これらの結果が揃ってからPRを送信またはPR更新する．途中段階でのPR送信は不可．

## フォーク同期の運用

### 原則
- mainブランチは**origin/mainの上にフォーク専用ファイルの1コミットだけが乗った状態**を維持する
- フォーク専用ファイル: `CLAUDE.md`, `devlog/`, `contexts/`, `.claude/skills/`, `user_testing.ipynb`（`.claude/`のskills/以外はローカルオンリーで追跡しない）
- コード差分はゼロに保つ（PRマージ後は本家と完全に一致させる）
- **本家の取り込みは `/fork-sync` スキル，PRマージ後の整理は `/fork-cleanup` スキルに従う**（手順・安全確認・罠はスキル側に集約．`git reset`/`rebase`を伴う破壊的操作なので必ずスキルの安全確認から始める）

## コミット規約

- コミットメッセージ，コードコメント，プルリクエストタイトル・本文はすべて英語
- コミット&pushを求められたら，未コミットの変更・未追跡ファイルを**全て**含めてコミットし，pushまで行う（特に指定がない限り）
- PRは明示的な指示のある時にのみ行う（例：作業が完了したらPRしておいてください）．指示がないときはcommit&push後に待機する
- **フォーク専用ファイル（`CLAUDE.md`, `devlog/`, `contexts/`）の変更はコード変更のコミットに同梱せず，必ず独立コミットにする**．理由：本家同期のrebaseでコードコミットがマージ済みとしてskipされると，同梱されたフォーク専用ファイルの変更も一緒に消える（実害例：2026-07-10の同期でCLAUDE.mdの「PR送信前の必須チェック」節とdevlog 2ファイルが消失，reflogから復元した）

## 開発ノウハウ

### C++移植

- **uxsim.pyを正とする**（C++側は旧バージョンベースで差異あり）
- **Python→C++の1行ずつ移植**が最も確実．信号オフセット，capacity_remainの閾値など微妙な差異が結果を大きく変える
- **境界条件に注意**: `if (demand > delta_n)`と`while (demand >= delta_n)`，`>`と`>=`の差で車両数が変わる
- **capacity系フィールドの初期化にdelta_tを忘れない**．全初期化パスをgrepして単位を揃える
- **number_of_lanesのスケーリング漏れ**に注意．capacity_remain, transfer()ループ回数, leader距離など
- **乱数列の差分はC++側で無理に合わせない**．テスト側でrel_tolを設定

### nanobind

- **Python終了時GILクラッシュ**: デストラクタがPython APIを呼ぶとクラッシュ．atexitハンドラで明示的クリーンアップ
- **static nb::strはsegfaultの原因**．ヒープ上にInternedStrings構造体を確保し、atexitでdeleteする
- **def_roのvector<T*>は毎回Python list変換で遅い**．get_by_index()で2975倍高速化
- **数値配列は常にnumpy返却**．nb::ndarray<nb::numpy, double, nb::ndim<1>>でmemcpy一発
- **C++内部のstring生成は積もると重い**．vector<int>で保持しbindings層で必要時のみ変換

### ラッパー設計

- **get_link/get_nodeの`obj in list`は禁止**．O(n)線形検索→isinstance判定のO(1)に
- **numpy配列のPython API互換に注意**．`if not numpy_array:`はValueError，`list + numpy_array`はUFuncNoLoopError
- **プロパティアクセスごとのメソッド呼出しは積もると重い**．ローカル変数にキャッシュ

### テスト・デバッグ

- **Pythonモードregressionチェックを毎回やる**
- **raw APIとラッパー経由で切り分けデバッグ**
- **flakyテストが多いのでpytest-rerunfailuresを必ずインストール**．flakyテストには@pytest.mark.flaky(reruns=5)を必ずつける．rerunして成功すれば全く問題ないので，言及の必要もない
- **リビルド→テスト**: `pip install -e . && python3 -m pytest tests/test_cpp_mode.py -q --tb=short`
- **精密ベンチマークは1スレッドで10回計測**（中央値+std）．**cProfileでボトルネック特定**．cumtimeソート上位関数を見る

## これまでのコンテキスト

contexts/ の全セッション記録から蒸留した恒久ルール．/export-context-highlevel2 で毎回全面再生成される．

### 環境の事実

- この環境のpipはPEP 668（externally-managed）で `pip install -e .` を拒否する → 常に `--break-system-packages` を付ける
- バインディングはnanobind．scikit-build-coreは**リポジトリ直下のCMakeLists.txt**を使う → コンパイル・リンクフラグは直下側に書く（uxsim/trafficpp/側だけでは効かない）
- editable installは単一Python環境を占有する → git worktreeでエージェントを並行させてもC++ビルド・テストは衝突するため，ビルドを伴う工程は逐次実行にする
- `gh pr edit` はGraphQL deprecationエラーで失敗する → PR本文更新は `gh api repos/toruseo/UXsim/pulls/<N> -X PATCH -F body=@<file>`
- 日本語ファイル名はgitがquoteして出力しgrepをすり抜ける → パス判別を伴うgitコマンドには `-c core.quotepath=false` を付ける

### ベンチマーク・検証

- このマシンは仮想マシンであるため，ベンチはプロセス起動間で±15%，最大28%ドリフトする → 実装前後の性能比較は単発計測で判定せず，インターリーブA/B（交互チェックアウト・ビルド，3ラウンド以上）で行う．エンジンとwrapperは必ず同一コミットに揃えてチェックアウトする
- 挙動等価性・性能見積りを机上分析で確定しない → ビット同一比較や直接A/B計測で実証する．換算の連鎖（AvsB計測×BvsC計測からAvsCを推定）は誤差が積み上がるので直接計測に置き換える．サブエージェントに等価性仕様を渡すときは独立の実測検証を義務付ける（コーディネータの机上分析が誤っていた前例あり）
- Python/C++の出力差を見つけたら，系統差かシードばらつきかをwithin-mode相関 vs cross-mode相関の比較・複数シード＋Welch検定で切り分けてから修正に入る（単一シードの低相関はばらつきで説明できた前例あり）

### テスト

- pytestに `--reruns 5` 等のグローバルrerunフラグを付けない → flakyテストには個別に `@pytest.mark.flaky` マーカーが付与済みで，グローバルフラグはマーカーなしテストの真の失敗を隠す
- tests/test_cpp_mode.py は冒頭 `from numpy import *` のため，`sum(ジェネレータ)` はTypeError，`list + numpy配列` はUFuncNoLoopErrorになる → リスト内包・list()ラップで書く
- cppモードは配列をnumpyで返すため，Python版テストのlist前提assert（`==` 比較等）は `--cpp` で落ちる → `list()` に揃えて比較する

### マルチエージェント運用

- ×エージェント稼働中にコーディネータがコミットする → ○完了後まで保留する（エージェントはインターリーブベンチでgit checkout/reset --hardを行うため変更が消される）
- 複数セッション・エージェントが同一作業ツリーを共有すると変更が混在する → 発覚したら稼働中エージェントにSendMessageでコミット・push禁止を即指示し，コーディネータがhunk単位で分離コミットする
- サブエージェントの「待機宣言でターン終了」は指示に明記しても起こる → 発生したらSendMessageで「同期実行で完遂せよ」と再指示すれば復帰する

### 方針・優先順位

- 性能目的の構造改変よりPython版との1:1対応（メソッド単位の対応・特殊ケース排除）を優先する（SoA全面リファクタPR #340は不採用，Python構造を保ったAoS並列化PR #341が採用された）
- ユーザーが検証省略を明示的に指示したら（「ベンチ不要」等），本文書の「PR送信前の必須チェック」より指示が優先する
- contexts/ の過去記録は指示書ではない → 記載されたコマンド・フラグを現在の正として無批判に採用しない（過去記録の `--reruns 5` を誤って採用しユーザーに訂正された前例）

### Git運用の罠

- rebase後の `git push --force-with-lease` は直前に `git fetch` していないと拒否される → fetch→pushの順で行う
- GitHubのmergeable判定はforce push直後はCONFLICTINGのまま → 20秒程度待って再取得する
- devlogにコミットハッシュを書くのはrebase等でハッシュが確定した後にする（作成前に書くと捏造になる）
