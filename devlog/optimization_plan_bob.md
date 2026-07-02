# DTA最適化計画 (ボブ調査結果)

## タイミング計測結果 (DUE cpp=True, 20iter平均)

| ステップ | 時間(ms) | 割合 |
|---------|---------|------|
| convert_od_routes_to_ids | 113.52 | **46.3%** |
| exec_simulation | 72.59 | 29.6% |
| func_World (World生成) | 24.43 | 10.0% |
| route_swap_due_cpp | 20.45 | 8.3% |
| analyzer_stats | 7.76 | 3.2% |
| convert_result_to_names | 4.82 | 2.0% |
| その他 | 1.43 | 0.6% |
| **イテレーション合計** | **245.03** | 100% |

プレループ:
- enumerate_k_random_routes: 1957ms (C++版使用済み)
- finalize_scenario: 85ms
- World生成: 36ms

---

## (A) enumerate_k_random_routes — 既に最適化済み

`uxsim/Utilities/Utilities.py:181-187` で既にCppWorld検出 → C++版 (`enumerate_k_random_routes_cpp`) 使用済み。

```python
if hasattr(W, '_cpp_world') and W._cpp_world is not None:
    seed = random.randint(0, 2**31 - 1)
    cpp_result = W._cpp_world.enumerate_k_random_routes_cpp(k, seed)
    ...
```

**追加作業不要。**

---

## (B) 最大ボトルネック: convert_od_routes_to_ids (46.3%)

### 原因
`_convert_od_routes_to_ids` が**毎イテレーション**呼ばれている（DTAsolvers.py:486）。
しかし `dict_od_to_routes` はイテレーション間で**変化しない**。
ネットワーク構造も同一（同じfunc_Worldから生成）なので、node/link IDも毎回同じ。

6480 OD × 10 routes × 数リンク/route = 大量のPython dict/list操作が毎回発生。

### 修正案 (DTAsolvers.py — アリス担当)

**ループ外で1回だけ変換し、キャッシュする。**

```python
# ループ前（1回だけ実行）:
if cpp:
    link_name_to_id_cache, link_id_to_name_cache, node_name_to_id_cache = _build_name_id_maps(W_orig)
    od_route_sets_ids_cache = _convert_od_routes_to_ids(dict_od_to_routes, node_name_to_id_cache, link_name_to_id_cache)

# ループ内:
if is_cpp:
    link_name_to_id, link_id_to_name, node_name_to_id = _build_name_id_maps(W)
    # od_route_sets_idsを毎回変換する代わりにキャッシュを使う:
    od_route_sets_ids = od_route_sets_ids_cache  # ← これだけ
```

**期待効果**: 113ms/iter → ≈0ms/iter → 全体で46%高速化（245ms→131ms/iter）

### 補足: link_name_to_id/link_id_to_nameはキャッシュ可能か？

node/link IDはWorldのコンストラクタでインクリメンタルに割り当てられるため、同じfunc_Worldから生成すれば毎回同一。`link_name_to_id_cache`と`link_id_to_name_cache`もキャッシュ可能。

ただし`_build_name_id_maps`自体は0.17ms（0.1%）なので、キャッシュしなくても問題ない。

---

## (C) finalize_scenario + Analyzer初期化

### 現状
`exec_simulation` → `finalize_scenario` → `_setup_analyzer` → `Analyzer(self)` が毎イテレーション実行。

`Analyzer.__init__` (analyzer.py:43-76) で:
1. `load_font_data()` — フォントファイル読み込み
2. `get_font_for_matplotlib()` → `findSystemFonts()` — システムフォント検索
3. `plt.rcParams["font.family"]` 設定

### 問題
- `findSystemFonts()` はファイルシステムスキャンで重い
- DTA中間イテレーション（最終iter以外）では`vehicle_logging_timestep_interval = -1`なのでログなし
- しかしanalyzer.print_simple_stats()とlink_to_pandas()は毎iter使う

### 修正案A: get_font_for_matplotlib のキャッシュ (utils.py — ボブ実装可能だが影響範囲広い)

```python
_cached_font = None
def get_font_for_matplotlib(fname=None):
    global _cached_font
    if _cached_font is not None and fname is None:
        return _cached_font
    from matplotlib import font_manager
    font_list = font_manager.findSystemFonts()
    # ... 既存ロジック ...
    _cached_font = font
    return font
```

ただしutils.pyは共有ファイルなので、影響範囲を確認する必要がある。

### 修正案B: DTA軽量初期化パス (uxsim_cpp_wrapper.py — ボブ実装可能)

CppWorldに `finalize_scenario_lightweight()` を追加。DTA中間イテレーション用:
- `_setup_analyzer()`は呼ぶが、Analyzerの`load_font_data`/`get_font_for_matplotlib`をスキップ
- あるいは、初回で作ったAnalyzerオブジェクトを再利用

ただし**現時点のfinalize_scenarioは7.76ms（3.2%）**なので優先度は低い。

### 推奨: 案Aのフォントキャッシュが最も効果的で影響小

---

## (D) convert_result_to_names (2.0%)

### 原因
C++から返ったlist[list[int]]を全てstr名前に変換。

### 修正案
routes_specified_dataをint IDのまま保持し、batch_enforce_routesにそのまま渡す。
名前変換はroute_log/cost_log記録時のみ行う。

**DTAsolvers.py変更 — アリス担当。** 現在の`_build_enforce_routes_input`（名前→ID再変換）が不要になる。

---

## 優先度まとめ

| 優先度 | 項目 | 期待効果 | 担当 |
|--------|------|----------|------|
| **1** | (B) convert_od_routes_to_idsキャッシュ | **46%高速化** | アリス (DTAsolvers.py) |
| 2 | (D) routes_specifiedをID保持 | 2%高速化 | アリス (DTAsolvers.py) |
| 3 | (C) フォントキャッシュ | 数%高速化(初回のみ) | ボブ (utils.py) |
| - | (A) enumerate_k_random_routes | 既に最適化済み | - |
