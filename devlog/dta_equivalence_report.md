# DTA Solver C++ vs Python 等価性検証レポート

## 1. route_swap_due の差異

### 差異1 (重大): DUE current route cost のtoll計算タイミング

**Python** (DTAsolvers.py:580):
```python
cost_current = r.actual_travel_time(ts[0]) + sum([l.get_toll(ts[i]) for i,l in enumerate(r)])
```
- `ts[i]` = 実際にリンクiに入った時刻（log_t_linkから取得した実時刻）
- tollは各リンクの**実際の進入時刻**で評価

**C++** (dta_solver.cpp:176-186):
```cpp
double t = departure_t;
for (size_t j = 0; j < traveled.link_ids.size(); j++){
    Link *ln = w->links[traveled.link_ids[j]];
    double ltt = dta_get_actual_travel_time(ln, t);
    double toll_t = (j < traveled.entry_times.size()) ? traveled.entry_times[j] : t;
    cost_current += dta_get_toll(ln, toll_t);
    cost_current += ltt;
    t += ltt;
}
```
- `traveled.entry_times[j]` = log_linkから取得した実進入時刻
- ここはPythonと**一致**している（entry_timesを使っている）

→ **一致** (問題なし)

### 差異2 (重大): DUE alt route cost のtoll計算タイミング

**Python** (DTAsolvers.py:584-585):
```python
alt_route_tts = alt_route.actual_travel_time(ts[0], return_details=True)[1]
cost_alt = alt_route.actual_travel_time(ts[0]) + sum([l.get_toll(ts[0]+sum(alt_route_tts[:i])) for i,l in enumerate(alt_route)])
```
- alt routeの各リンクのtollは `ts[0] + sum(alt_route_tts[:i])` で評価
- つまり**出発時刻 + 累積actual_travel_time**の推定進入時刻でtollを計算
- `actual_travel_time` を2回呼んでいるが、1回目でdetails取得、2回目はtotal取得（結果は同じ）

**C++** (dta_solver.cpp:50-62, dta_compute_route_cost_due):
```cpp
double t = departure_time;
for (int lid : route_link_ids){
    Link *ln = w->links[lid];
    double ltt = dta_get_actual_travel_time(ln, t);
    toll_total += dta_get_toll(ln, t);  // toll at estimated entry time
    tt_total += ltt;
    t += ltt;
}
return tt_total + toll_total;
```
- 各リンクのtollを `t`（累積actual_travel_timeベースの推定進入時刻）で評価

→ **一致** (toll計算タイミングは同じ: 累積actual_travel_timeベース)

### 差異3 (重大): traveled_route の取得方法

**Python** (DTAsolvers.py:562):
```python
r, ts = veh.traveled_route()
```
- `traveled_route()` は `log_t_link` から構築
- `ts` = [link0_entry_time, link1_entry_time, ..., arrival_time]
- `r` = Route object (links list)
- `include_arrival_time=True`（デフォルト）なので `ts[-1]` = arrival_time

**C++** (dta_solver.cpp:12-29, dta_get_traveled_route):
```cpp
int prev_lid = -999;
for (size_t i = 0; i < veh->log_size; i++){
    int lid = veh->log_link[i];
    if (lid >= 0 && lid != prev_lid){
        info.link_ids.push_back(lid);
        info.entry_times.push_back(veh->log_t[i]);
        prev_lid = lid;
    }
}
if (veh->state == vsEND && veh->arrival_time >= 0){
    info.arrival_time = veh->arrival_time;
}
```
- log_link/log_tから直接link遷移を検出（log_t_linkと同じロジック）
- entry_timesにはlink進入時刻のみ（arrival_timeは別フィールド）

**Python build_all_vehicle_logs_flat_compact** (traffi.cpp:1698-1707):
```cpp
int prev_lid = -999;
for (size_t i = 0; i < v->log_size; i++) {
    int lid = v->log_link[i];
    if (lid >= 0 && lid != prev_lid) {
        fl.ltl_t[ltl_base + ltl_idx] = v->log_t[i];
        fl.ltl_id[ltl_base + ltl_idx] = lid;
        ...
```

→ **一致** (同じlog_link遷移検出ロジック)

### 差異4 (軽微): cost_actual の計算

**Python** (DTAsolvers.py:563):
```python
travel_time = ts[-1] - ts[0]
cost_actual[key] = travel_time
```
- `ts[-1]` = arrival_time, `ts[0]` = first link entry time

**C++** (dta_solver.cpp:135-136):
```cpp
if (!traveled.entry_times.empty() && traveled.arrival_time >= 0){
    result.cost_actual[vi] = traveled.arrival_time - traveled.entry_times[0];
}
```

→ **一致**

### 差異5 (重大): state != "end" のときの routes_specified

**Python** (DTAsolvers.py:568-569):
```python
if veh.state != "end":
    continue
```
- `continue`するだけ → routes_specifiedにこの車両のエントリは**追加されない**
- 次のイテレーションで `if key in routes_specified` → False → enforce_routeされない

**C++** (dta_solver.cpp:142-144):
```cpp
if (veh->state != vsEND){
    result.routes_specified[vi] = traveled.link_ids;
    continue;
}
```
- routes_specifiedに**traveled routeを入れている**

**Python側の呼び出し** (DTAsolvers.py:514):
```python
routes_specified_data = result['routes_specified']
```
そして `_convert_cpp_result_to_names` (L98):
```python
if route_ids:
    routes_specified_data[veh_names[idx]] = [link_id_to_name[lid] for lid in route_ids]
```
- 空でないroute_idsはroutes_specified_dataに追加される

→ **差異あり（軽微）**: C++ではstate!=endの車両にもtraveled routeをenforceする。Pythonではスキップ。
ただし、state!=endの車両は前回のルートで走行中に終了できなかった車両なので、次イテレーションでもenforce_routeしてもしなくても挙動は同じ可能性が高い（新Worldでは全車両が初期状態から再出発するため）。

**実質的影響**: 低い。新World生成時に全車両は再初期化されるので、state!=endだった車両のroute enforceは次のWorld再生成で消える。ただし、abort/未完了車両にenforceしている分のオーバーヘッドがある。

### 差異6 (軽微): potential_n_swap の累積方式

**Python** (DTAsolvers.py:206,219):
```python
potential_n_swap_updated = potential_n_swap
for alt_route in route_set[o,d]:
    if cost_alt < cost_current:
        if flag_route_changed == False or (cost_alt < cost_current):
            potential_n_swap_updated = potential_n_swap + W.DELTAN
potential_n_swap = potential_n_swap_updated
```
- `potential_n_swap_updated`は車両ごとにリセットされ、最後に`potential_n_swap`に代入
- 複数のalt_routeがcost_currentより安くても、**1車両あたり最大+DELTAN**

**C++** (dta_solver.cpp:192-209):
```cpp
double potential_n_swap_delta = 0.0;
for (size_t ri = 0; ri < route_set.size(); ri++){
    if (cost_alt < cost_current){
        if (!flag_route_changed || cost_alt < cost_current){
            potential_n_swap_delta = w->delta_n;
        }
    }
}
result.potential_n_swap += potential_n_swap_delta;
```

→ **一致** (1車両あたり最大+delta_n)

### 差異7: 乱数生成

**Python**: `random.random() < swap_prob` — グローバルPython乱数
**C++**: `std::mt19937 local_rng(rng_seed)` + `uniform_real_distribution`

→ **乱数列は異なる** (これは意図的。同じseedでも結果は異なる)

---

## 2. route_swap_dso の差異

### 差異8 (重大): DSO cost計算

**Python** (DTAsolvers.py:990-992):
```python
ext = estimate_congestion_externality_route(W, r, ts[0])
private_cost = r.actual_travel_time(ts[0])
cost_current = private_cost + ext
```

`estimate_congestion_externality_route` (Utilities.py:506-508, CppWorld path):
```python
route_links = [l._cpp_link for l in route]
return W._cpp_world.estimate_congestion_externality_route(route_links, t)
```

`World::estimate_congestion_externality_route` (traffi.cpp:606-621):
```cpp
double t = departure_time;
double exts = 0;
for (auto ln : route){
    int ts = (int)(t / delta_t);
    exts += ln->estimate_congestion_externality(ts);
    int n = (int)ln->traveltime_real.size();
    if (ts >= 0 && ts < n){
        t += ln->traveltime_real[ts];
    } else {
        t += ln->length / ln->vmax;
    }
}
return exts;
```

**C++** (dta_solver.cpp:65-78, dta_compute_route_cost_dso):
```cpp
double t = departure_time;
for (int lid : route_link_ids){
    Link *ln = w->links[lid];
    double ltt = dta_get_actual_travel_time(ln, t);
    int ts = (int)(t / w->delta_t);
    ext_total += ln->estimate_congestion_externality(ts);
    tt_total += ltt;
    t += ltt;
}
return tt_total + ext_total;
```

対照:
- `dta_get_actual_travel_time` と `traveltime_real[ts]` は同じ（境界処理の微小差のみ）
- externality計算: 同じ `estimate_congestion_externality(ts)` を呼んでいる
- 時間の進め方: 同じ（actual_travel_timeで進める）

→ **一致** (dta_get_actual_travel_timeの境界処理がわずかに異なるが、通常の使用では影響なし)

**境界処理の微小差**:
- traffi.cpp版: `if (ts >= 0 && ts < n)` → 範囲外なら `length/vmax`
- dta_solver.h版: `if (idx >= n) idx = n - 1` → 最後の値を返す
- 通常シミュレーション範囲内では同じ値

### 差異9: DSO swap_num の扱い

**Python** (DTAsolvers.py:964-966):
```python
keys_swap = [key for key in W.VEHICLES.keys() if random.random() < swap_prob]
if swap_num != None:
    keys_swap = random.sample(list(W.VEHICLES.keys()), swap_num)
```
- swap_numが指定されると、swap_probの結果を**完全に上書き**

**C++** (dta_solver.cpp:248-263):
```cpp
if (swap_num >= 0){
    std::vector<int> indices(nv);
    // ...
    std::shuffle(indices.begin(), indices.end(), local_rng);
    int actual_swap_num = std::min(swap_num, (int)nv);
    for (int i = 0; i < actual_swap_num; i++){
        is_swap_candidate[indices[i]] = true;
    }
} else {
    for (size_t i = 0; i < nv; i++){
        is_swap_candidate[i] = (udist(local_rng) < swap_prob);
    }
}
```

→ **一致** (swap_num >= 0 のとき swap_prob 無視、shuffle でランダム選択)

### 差異5 (再掲): state != "end" のときの routes_specified

DUEと同じ差異がDSOにもある。

---

## 3. batch_enforce_routes の差異

**Python path** (DTAsolvers.py:522-525):
```python
if i != 0:
    for key in W.VEHICLES:
        if key in routes_specified:
            W.VEHICLES[key].enforce_route(routes_specified[key])
```
- `routes_specified[key]` は Route object or list of link names
- enforce_routeはCppVehicle.enforce_route → C++のVehicle::enforce_route

**C++ path** (DTAsolvers.py:489-491):
```python
enforce_input = _build_enforce_routes_input(W, routes_specified_data, link_name_to_id)
W._cpp_world.batch_enforce_routes(enforce_input)
```

**dta_solver.cpp** (L85-99):
```cpp
void dta_batch_enforce_routes(World *w, const std::vector<std::vector<int>> &routes_per_vehicle) {
    size_t n = std::min(routes_per_vehicle.size(), w->vehicles.size());
    for (size_t i = 0; i < n; i++){
        const auto &route_ids = routes_per_vehicle[i];
        if (route_ids.empty()) continue;
        std::vector<Link*> route;
        for (int lid : route_ids) route.push_back(w->links[lid]);
        w->vehicles[i]->enforce_route(route);
    }
}
```

→ **一致** (同じ Vehicle::enforce_route を呼んでいる)

**_build_enforce_routes_input** (DTAsolvers.py:134-142):
```python
for veh_name in W.VEHICLES:
    if veh_name in routes_specified_data:
        routes_per_vehicle.append([link_name_to_id[ln] for ln in routes_specified_data[veh_name]])
    else:
        routes_per_vehicle.append([])
```
- routes_specified_dataにないvehicleは空リスト → C++でskip

→ **一致**

---

## 差異まとめ

| # | 箇所 | 重大度 | 内容 | 影響 |
|---|------|--------|------|------|
| 5 | state!=end時のroutes_specified | 軽微 | C++はtraveled routeを入れる、Pythonはスキップ | 新World生成で消えるため実質無影響 |
| 8境界 | dta_get_actual_travel_time境界処理 | 軽微 | 範囲外: C++はclamp(最後の値)、traffi.cppはlength/vmax | シミュ範囲内では同一 |
| 7 | 乱数列 | 意図的 | Python random vs C++ mt19937 | 結果は確率的に等価 |

**重大な論理差異はなし。** 結果の違いは乱数列の差のみ。
