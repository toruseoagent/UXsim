---
name: fork-sync
description: 本家（origin=toruseo/UXsim）mainの最新をフォーク作業リポジトリのmainに取り込む．「本家と同期」「本家の最新を取り込んで」「pull origin」等で使用．PRマージ直後のクリーンアップにはfork-cleanupを使う
---

# 本家との同期（fork-sync）

mainブランチを「origin/mainの上にフォーク専用1コミット（＋作業コミット）が乗った状態」に保ったまま，本家の最新を取り込む．

フォーク専用ファイル: `CLAUDE.md`, `devlog/`, `contexts/`, `.claude/skills/`, `user_testing.ipynb`（存在する場合）．`.claude/`のskills/以外はローカルオンリー（未追跡のまま）

## 手順

```bash
# 1. 安全確認（最重要．rebaseは破壊的で，未pushコミットはreflogでしか復旧できない）
git status                     # クリーンであること（untrackedの .claude/, tmp/ は無視してよい）
git log --oneline origin/main..HEAD   # フォーク専用＋把握済みの作業コミットのみであること

# 2. 本家の最新を取り込む
git fetch origin main
git pull origin main --rebase

# 3. コンフリクトがあれば解消（コード側の衝突は，本家に同内容が既に入っていないか確認してから解消する）

# 4. コード差分が残っていないか確認（フォーク専用以外が0件であること）
git -c core.quotepath=false diff origin/main..HEAD --name-only | grep -vE '^(CLAUDE\.md|devlog/|contexts/|\.claude/skills/|user_testing\.ipynb)'

# 5. push（rebase後は必ずfetch fork してからでないとforce-with-leaseが失敗する）
git fetch fork main
git push fork main --force-with-lease
```

## 罠

- **手順4のquotepath無効化は必須**．日本語ファイル名（contexts/等）はgitがquoteして出力するため，`"` で始まる行がgrepフィルタをすり抜けて誤検出する
- **force-with-leaseはローカルrebase後にfetchしていないと失敗する**（リモート追跡refが古いため）
- 別セッションが並行作業している可能性がある．手順1でfork/mainと差分があれば（`git log main..fork/main`），先に取り込むかユーザーに確認する
- 本家でuxsim.pyに新機能が入っていたら，C++追従タスクの候補としてユーザーに報告する（勝手に着手しない）
