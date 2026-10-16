---
name: fork-cleanup
description: 本家PRマージ後のフォークmainクリーンアップ．mainを「origin/main＋フォーク専用1コミット」に整理し，マージ済みPRブランチを削除する．「マージされたのでクリーンアップして」「フォークのmainを同期・クリーンアップ」等で使用
---

# PRマージ後のクリーンアップ（fork-cleanup）

本家PRのマージ後，フォークのmainに残るcherry-pick残骸・rebase重複コミットを整理し，「N commits ahead」を実態（フォーク専用ファイルのみ）と一致させる．

フォーク専用ファイル: `CLAUDE.md`, `devlog/`, `contexts/`, `.claude/skills/`, `user_testing.ipynb`（存在する場合）．`.claude/`のskills/以外はローカルオンリー（未追跡のまま）

## ⚠ 事前の安全確認（省略不可）

`git reset` は破壊的操作．未コミットの変更は`git add`で拾わなければ消え，コミット済み未pushの変更は`git reflog`でしか復旧できない．

```bash
git status                     # クリーンであること（untrackedの .claude/, tmp/ は無視してよい）
git fetch origin main && git fetch fork main
gh pr view <PR番号> --repo toruseo/UXsim --json state,mergedAt   # MERGEDであること
git log --oneline main..fork/main    # 別セッションのpushがないこと（あれば先に取り込む）
git -c core.quotepath=false diff origin/main..main --name-only | grep -vE '^(CLAUDE\.md|devlog/|contexts/|\.claude/skills/|user_testing\.ipynb)'
# → フォーク専用以外が表示されたら未マージの作業の可能性．クリーンアップを中断しユーザーに確認
```

## 手順

```bash
# 1. フォーク専用ファイルの最新状態の参照元を控える
PREV=$(git rev-parse --short main)

# 2. mainを本家に一致させ，フォーク専用ファイルを1コミットで載せ直す
git checkout main
git reset --hard origin/main
git checkout $PREV -- CLAUDE.md devlog/ contexts/ .claude/skills/
# user_testing.ipynb が $PREV に存在する場合はそれも含める
git commit -m "Add fork-only files (CLAUDE.md, devlog/, contexts/)"

# 3. 差分検証（フォーク専用以外が0件であること．quotepath無効化必須）
git -c core.quotepath=false diff origin/main..main --name-only | grep -cvE '^(CLAUDE\.md|devlog/|contexts/|\.claude/skills/|user_testing\.ipynb)'

# 4. push（fetch済みでないとforce-with-leaseが失敗する）
git push fork main --force-with-lease

# 5. マージ済みPRブランチの削除（ローカル・fork両方）
git push fork --delete pr/<feature>
git branch -D pr/<feature>
```

## 付随作業

- devlogの完了報告にPRのマージ状態（マージ日・squash/mergeコミットハッシュ）を追記してから手順2のコミットに含める
- 作業ブランチ（refactor/*等）にのみ存在するdevlog/contextsがないか確認し（`git diff main..<branch> --stat -- devlog/ contexts/`），あれば手順2で回収する
- 不採用に終わったブランチの記録を回収する場合は，不採用の旨と採用された代替（PR番号）を冒頭に追記する

## 罠

- **quotepath無効化は必須**．日本語ファイル名はgitがquoteして出力し，grepフィルタをすり抜ける
- **squashマージの場合，作業ブランチのコミットハッシュは本家に存在しない**．devlogにハッシュを書く際は本家側のハッシュを確認する
- マージ方式（merge/squash）はgh pr viewや`git log origin/main`で確認できる
