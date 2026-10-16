# 計画: Issue #358 World の deepcopy で起きる RecursionError の修正

## 目的

`run-notebook-demos-extra` の `test_demo_notebook_03en_pytorch` が 2026-08-10 以降，`copy.deepcopy(env.W)` で `RecursionError: maximum recursion depth exceeded` を出して失敗し続けている（toruseo/UXsim#358）。
原因を特定し，Python 版 UXsim 本体で直す。
Copilot の PR #359（再帰上限を一時的に引き上げる `World.__deepcopy__`）はクローズ済みで，本計画はそれを前提にしない。

## 原因（再現で確認済み）

`Vehicle.end_trip()`（`uxsim/uxsim.py:1230-1231`）は後続車の `leader` を `None` にするが，自分の `follower` を `None` にしない。
リンク間遷移（`uxsim/uxsim.py:366-368`）では両方を `None` にしている。
このため，同じリンクで続けてトリップを終えた車両が `follower` で数珠つなぎに残り，連鎖の長さは終了車両数に比例して伸びる。
`copy.deepcopy` は World のオブジェクトグラフを深さ優先で再帰的にたどるので，この連鎖1段につき約4フレーム（`deepcopy` → `_reconstruct` → `deepcopy(state)` → `_deepcopy_dict`）を消費し，既定の再帰上限1000を超える。
`World.copy()` と `World.save()` は dill の pickle を使うが，同じ理由で失敗する。

失敗したノートブックのトレースバックは `copy.py` のフレームだけで構成されている。
修正候補（`end_trip` で `s.follower = None` を追加）を monkeypatch で当てて，deepcopy が成功するのに必要な最小の再帰上限を二分探索で測った。
シナリオはノートブックの `TrafficSim.reset()` と同じ World で，信号は torch を使わずに機械的に操作した（`tmp/issue358/repro_deepcopy.py`）。

| シナリオ | 修正前 | 修正後 |
|---|---|---|
| 固定サイクル10秒 seed 1（遅れ 42.8 s，全8295トリップ完了） | 1186（失敗） | 300 |
| 1リンク・300小隊が全て終了（回帰テスト用の最小シナリオ） | 1226（失敗） | 50以下 |
| ランダム操作 40シード（中央値 / 最大） | 800 / 1048 | 656 / 842 |
| 固定サイクル 10〜60秒 各2〜3シード（範囲） | 636〜1034 | 300〜890 |

- 全ケースで TTT・平均遅れ・完了トリップ数は修正前と一致した。
- 1リンクの最小シナリオでは，修正前は `copy.deepcopy(W)` と `W.copy()` の両方が RecursionError になり，修正後は両方成功した。
- ノートブックのカーネル内では，セル開始時点で既に22フレームが積まれている（Python 3.12.13，ipykernel 7.3.0，nbclient 経由で実測）。ノートブック内で deepcopy に使える深さは約975になる。

## 「最近」失敗し始めた理由（未特定）

同じコミット f12c848 が 2026-08-03 の定期実行では成功し，2026-08-10 では失敗した。
コードは変わっていないので，CI の実行環境か乱数による結果の違いが引き金と考えられる。

- 2回の実行で，ランナーイメージ（ubuntu-24.04，20260720.247.2）と Python（3.12.13）は同じだった。
- パッケージの差分は numpy 2.5.1→2.5.2，tornado 6.5.7→6.5.8，nbformat 5.10.4→5.11.0，setuptools 83→84 など11件で，torch・gymnasium・ipykernel は同じ版だった。
- numpy 2.5.1 と 2.5.2 では，必要な再帰上限が全シードで1フレームも変わらなかった。
- tornado 6.5.7 と 6.5.8 では，カーネル内の基底フレーム数がどちらも22だった。
- 必要な深さはシミュレーション結果によって数百フレーム変わる。ノートブックは `seed=None` で需要を生成し，DQN にも乱数シードを固定していないので，CI の実行ごとに結果が異なる。
- 失敗した deepcopy の遅れは 71.4 s（本家 2026-08-10，エピソード27），103.0 s（本家 2026-10-16，エピソード23），286.5 s（フォーク 2026-10-16，エピソード3）だった。
- 2026-08-10 を境に成功が続く状態から失敗が続く状態へ変わった理由は，上の差分では説明できていない。修正はこの引き金に依存しない。

## 修正内容

### 1. `uxsim/uxsim.py` の `Vehicle.end_trip()`

リンク間遷移と同じく，トリップ終了時に自分の `follower` も解除する。

```python
        if s.follower != None:
            s.follower.leader = None
            s.follower = None
```

終了済み車両の `follower` を読む処理は `uxsim/` 内に無い（`leader`/`follower` を参照するのは `uxsim/uxsim.py` だけ）。
車両がリンクから外れるのはリンク間遷移（`uxsim/uxsim.py:361`）と `end_trip`（`uxsim/uxsim.py:1233`）の2か所だけである。

### 2. 回帰テスト `tests/test_other_functions.py`

既存の `test_world_save` と `test_world_copy` はシミュレーション前の World しか複製していない。
その直後に，シミュレーション後の World を複製するテストを1本追加する。

- シナリオは 2ノード・1リンク（長さ1000 m，自由流速度20 m/s），需要 0.5 台/s を0〜3000 s，`deltan=5`，`tmax=3600` とする。実行時間は約0.1秒である。
- 終了した全車両の `follower` が `None` であることを確認する。
- 既定の再帰上限のまま `copy.deepcopy(W)` と `W.copy()` が成功し，複製の `total_travel_time` が元と一致することを確認する。
- 修正前のコードではこのテストが RecursionError で失敗することを，実装時に確認する。

### 3. C++ エンジン `uxsim/trafficpp/traffi.cpp`（要判断）

C++ 側の `end_trip` 相当（`uxsim/trafficpp/traffi.cpp:912-914`）も `follower->leader = nullptr` だけで，自分の `follower` を解除していない。
C++ モードは deepcopy と pickle に対応していないので，この Issue の症状は出ない。
「uxsim.py を正とし1行ずつ対応させる」方針に合わせるなら `follower = nullptr;` を追加する。

## 検証計画

1. 修正前後でシミュレーション結果が変わらないことを確認する。ノートブックの World（ランダム操作と固定サイクル，複数シード），多車線のシナリオ（`number_of_lanes>1` では先行車が `vehicles[-number_of_lanes]` になる），既存テストのシナリオで，全車両の `log_t`・`log_x`・`log_v`・通過リンク・到着時刻を比較する。
2. 追加したテストが修正前に失敗し，修正後に通ることを確認する。
3. ローカルで `tests/test_other_functions.py` と `tests/test_cpp_mode.py` を実行する。Python モードの全テストはフォークへの push 後に GitHub Actions で確認する。
4. フォークへの push 後，`Run Notebooks demos (extra)` の `test_demo_notebook_03en_pytorch` が通ることを確認する。DQN の結果は実行ごとに変わるので，workflow_dispatch で3回程度実行する。
5. C++ 側も変更する場合は，`tests/test_cpp_mode.py` の全件通過に加えて，C++ モードの結果が変更前と一致することを確認する。

## 範囲外（報告のみ）

- 修正後も，渋滞中の待ち行列の `follower` 連鎖が `Vehicle.log_link` → `Link.vehicles` を介して複数リンクにまたがるため，World の deepcopy と pickle には数百フレームの深さが残る。今回の12ノードのネットワークでは最大890を測った。この深さはネットワークの規模と渋滞に応じて伸びるので，大きなネットワークでは修正後も RecursionError になりうる。対策の候補は，World の `__deepcopy__` で Node・Link・Vehicle の空オブジェクトを先に memo に登録してから各 `__dict__` を複製する方式である。これで再帰の深さが1オブジェクト内の入れ子の深さに抑えられると見込んでいるが，未検証である。pickle 側には別の設計が要る。
- 同じワークフローの `test_demo_notebook_04en_OpenStreetMap` は，overpass-api.de が 502 を返して失敗している。外部サーバー側の問題で，この Issue とは関係しない。
- `demos_and_examples/example_09en_congestion_pricing_highway_toll.py:74` に「copy.deepcopy does not work for some reason」というコメントがある。同じ原因かは確認していない。

## 不採用案

- 再帰上限を引き上げる方式（PR #359）は，`follower` の不整合を残したまま症状だけを隠し，必要な深さは終了車両数に比例して伸び続ける。`sys.setrecursionlimit` はプロセス全体の設定を変更する。`World.copy()` と `World.save()` の失敗も直らない。
- ノートブック側で `sys.setrecursionlimit` を上げる，または `deepcopy` を使わないように書き換える方式は，テストを通すだけで，利用者のコードでの失敗は残る。

## 要確認事項

1. フォークの CLAUDE.md は `uxsim/uxsim.py` を「編集しないファイル」としているが，この修正は `uxsim/uxsim.py` の変更を必要とする。
2. C++ 側の対応（修正内容3）を同じ PR に含めるか。
3. 範囲外の1項目目（深さの上限を抑える deepcopy）を今回扱うか，別 Issue にするか。
4. ノートブックのテストをローカルで実行するには `.venv` に torch と gymnasium を入れる必要がある。入れずに CI だけで確認するか。

## 再現

調査用スクリプトは git 管理外の `tmp/issue358/` にある。

```bash
cd /home/ubuntu/UXsim && source .venv/bin/activate
python -I tmp/issue358/repro_deepcopy.py measure 2 cycle10        # 修正前：seed 1 で必要な上限 1186
python -I tmp/issue358/repro_deepcopy.py measure-fixed 2 cycle10  # 修正後：seed 1 で 300
python -I tmp/issue358/measure_many.py fixed random 0 40          # 修正後のランダム操作40シード
```
