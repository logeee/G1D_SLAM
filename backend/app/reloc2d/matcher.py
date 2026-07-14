#!/usr/bin/env python3
"""2D 重定位核心:LaserScan ↔ 思岚 2D 占据栅格。纯 numpy/scipy(无 open3d)。

坐标约定
--------
- 位姿 pose = (x, y, theta):把 **激光/机体系** 点变换到 **地图(世界)系**:
      world = R(theta) @ p_laser + [x, y]
  即 (x, y, theta) 就是机器人在地图系下的位姿(思岚 set_pose 需要的量)。
- scan 点在激光系;地图障碍点在世界系。求 (x,y,theta) 使变换后的 scan 贴合地图障碍。

三种方法(与 D5G-beacon 一脉相承,降到 2D)
-----------------------------------------
- global : 无初值。似然场(EDT)上做 多分辨率 平移×旋转 相关性搜索 -> 2D ICP 精修。
- two_stage(给初值 center_xy):中心+圆环多位置 × 一圈 yaw 撒种,粗->精两段 2D ICP。
            初值来源(json 上次位姿 / 手动点击)只提供位置,朝向由 yaw 扫描搜。
"""
import math
import numpy as np

try:
    from scipy.spatial import cKDTree
    from scipy.ndimage import distance_transform_edt
except Exception as exc:  # pragma: no cover
    raise ImportError(f"reloc2d 需要 scipy(cKDTree / distance_transform_edt): {exc}")


OCC_THRESH = 50  # 占据栅格 >= 该值视为障碍(思岚:占据~100,空闲 0,未知 -1)


# ----------------------------------------------------------------------------
# 数据转换
# ----------------------------------------------------------------------------
def scan_to_points(scan, max_points=1200):
    """LaserScan payload -> 激光系 2D 点 [M,2]。剔除无效(None)束。"""
    if not scan:
        return np.zeros((0, 2), dtype=np.float64)
    ranges = scan.get("ranges") or []
    a0 = scan.get("angle_min")
    da = scan.get("angle_increment")
    if a0 is None or da is None or not ranges:
        return np.zeros((0, 2), dtype=np.float64)
    a0 = float(a0)
    da = float(da)
    pts = []
    for i, r in enumerate(ranges):
        if r is None:
            continue
        r = float(r)
        if not math.isfinite(r) or r <= 0.0:
            continue
        ang = a0 + i * da
        pts.append((r * math.cos(ang), r * math.sin(ang)))
    if not pts:
        return np.zeros((0, 2), dtype=np.float64)
    arr = np.asarray(pts, dtype=np.float64)
    if len(arr) > max_points:
        sel = np.linspace(0, len(arr) - 1, max_points).astype(int)
        arr = arr[sel]
    return arr


def _map_meta(mp):
    origin = mp.get("origin") or {}
    return {
        "res": float(mp.get("resolution") or 0.05),
        "ox": float(origin.get("x") or 0.0),
        "oy": float(origin.get("y") or 0.0),
        "oyaw": float(origin.get("yaw") or 0.0),
        "w": int(mp.get("width") or 0),
        "h": int(mp.get("height") or 0),
    }


def _occ_grid(mp):
    meta = _map_meta(mp)
    w, h = meta["w"], meta["h"]
    data = np.asarray(mp.get("data") or [], dtype=np.int16)
    if w <= 0 or h <= 0 or data.size != w * h:
        raise ValueError(f"占据栅格数据尺寸不符: data={data.size}, w*h={w * h}")
    grid = data.reshape(h, w)  # row-major:grid[row, col]
    return grid, meta


def occupancy_to_points(mp, occ_thresh=OCC_THRESH):
    """占据栅格 -> 世界系障碍点 [N,2](栅格中心)。"""
    grid, meta = _occ_grid(mp)
    rows, cols = np.where(grid >= occ_thresh)
    if len(rows) == 0:
        return np.zeros((0, 2), dtype=np.float64)
    res = meta["res"]
    lx = (cols.astype(np.float64) + 0.5) * res
    ly = (rows.astype(np.float64) + 0.5) * res
    return _local_to_world(np.stack([lx, ly], axis=1), meta)


def _local_to_world(local, meta):
    oyaw = meta["oyaw"]
    if abs(oyaw) > 1e-9:
        c, s = math.cos(oyaw), math.sin(oyaw)
        R = np.array([[c, -s], [s, c]], dtype=np.float64)
        world = local @ R.T
    else:
        world = local
    return world + np.array([meta["ox"], meta["oy"]], dtype=np.float64)


def _world_to_cell(pts, meta):
    """世界系点 -> (col, row, valid)。"""
    p = pts - np.array([meta["ox"], meta["oy"]], dtype=np.float64)
    oyaw = meta["oyaw"]
    if abs(oyaw) > 1e-9:
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        R = np.array([[c, -s], [s, c]], dtype=np.float64)
        p = p @ R.T
    res = meta["res"]
    col = np.floor(p[:, 0] / res).astype(np.int64)
    row = np.floor(p[:, 1] / res).astype(np.int64)
    valid = (col >= 0) & (col < meta["w"]) & (row >= 0) & (row < meta["h"])
    return col, row, valid


# ----------------------------------------------------------------------------
# 几何工具
# ----------------------------------------------------------------------------
def _rot(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=np.float64)


def _transform(pts, x, y, theta):
    return pts @ _rot(theta).T + np.array([x, y], dtype=np.float64)


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ----------------------------------------------------------------------------
# 2D ICP(点到点,KD 树最近邻 + SVD 闭式解)
# ----------------------------------------------------------------------------
def icp_2d(src, tgt_pts, tgt_tree, init, max_iter=40, max_corr=0.5, tol=1e-5):
    """src(激光系) 对 tgt_pts(世界系) 做 point-to-point ICP。
    init=(x,y,theta)。返回 dict{x,y,theta,rmse,fitness,inliers,iters}。"""
    x, y, theta = float(init[0]), float(init[1]), float(init[2])
    n = len(src)
    if n < 10:
        return {"x": x, "y": y, "theta": theta, "rmse": float("inf"),
                "fitness": 0.0, "inliers": 0, "iters": 0}
    prev_rmse = float("inf")
    inliers = 0
    rmse = float("inf")
    it = 0
    for it in range(1, max_iter + 1):
        cur = _transform(src, x, y, theta)
        dist, idx = tgt_tree.query(cur, k=1)
        mask = dist < max_corr
        inliers = int(mask.sum())
        if inliers < 10:
            break
        P = cur[mask]
        Q = tgt_pts[idx[mask]]
        muP = P.mean(axis=0)
        muQ = Q.mean(axis=0)
        H = (P - muP).T @ (Q - muQ)
        U, _, Vt = np.linalg.svd(H)
        d = np.sign(np.linalg.det(Vt.T @ U.T))
        D = np.array([[1.0, 0.0], [0.0, d]], dtype=np.float64)
        R = Vt.T @ D @ U.T                     # 把 P 对到 Q 的增量旋转
        t = muQ - R @ muP
        # 把增量 (R,t) 复合到当前绝对位姿:new = (R@R0) src + (R@t0 + t)
        dtheta = math.atan2(R[1, 0], R[0, 0])
        new_xy = R @ np.array([x, y], dtype=np.float64) + t
        x, y = float(new_xy[0]), float(new_xy[1])
        theta = _wrap(theta + dtheta)
        rmse = float(math.sqrt(np.mean(dist[mask] ** 2)))
        if abs(prev_rmse - rmse) < tol:
            break
        prev_rmse = rmse
    fitness = inliers / float(n)
    return {"x": x, "y": y, "theta": theta, "rmse": rmse,
            "fitness": fitness, "inliers": inliers, "iters": it}


# ----------------------------------------------------------------------------
# 双阶段 ICP:给定初值位置,多起点(圆环 × yaw)粗->精
# ----------------------------------------------------------------------------
def two_stage_icp(src, map_pts, center_xy, init_yaw=None,
                  crop_r=8.0, ring_r=0.5, n_ring=6, n_yaw=12,
                  coarse_corr=1.0, fine_corr=0.3, top_k=6):
    """以 center_xy 为中心撒多起点,粗体素大 corr 排序 -> top_k 细 corr 精修取最优。
    朝向绕竖轴扫满一圈(n_yaw);init_yaw 若给定则额外加一个精确起点。"""
    center = np.asarray(center_xy[:2], dtype=np.float64)

    # 裁地图:只留中心附近 crop_r,加快 KD 树
    d = np.linalg.norm(map_pts - center, axis=1)
    local = map_pts[d < crop_r]
    if len(local) < 50:
        local = map_pts[d < max(crop_r * 2.0, 20.0)]
    if len(local) < 50:
        raise ValueError("初值附近地图障碍点过少,可能初值偏差过大或选到地图外")
    tree = cKDTree(local)

    # 起点:中心 + 圆环位置;yaw 均布一圈
    positions = [center]
    if ring_r > 1e-6 and n_ring > 0:
        for k in range(n_ring):
            th = 2.0 * math.pi * k / n_ring
            positions.append(center + ring_r * np.array([math.cos(th), math.sin(th)]))
    yaws = [2.0 * math.pi * k / n_yaw for k in range(max(1, n_yaw))]
    if init_yaw is not None:
        yaws.append(float(init_yaw))

    # 粗段:少迭代 + 大 corr,给所有起点打分
    coarse = []
    for pos in positions:
        for yaw in yaws:
            r = icp_2d(src, local, tree, (pos[0], pos[1], yaw),
                       max_iter=15, max_corr=coarse_corr)
            coarse.append(r)
    coarse.sort(key=lambda r: (-r["fitness"], r["rmse"]))

    # 精段:top_k 细 corr 精修
    refined = []
    for r in coarse[:top_k]:
        rr = icp_2d(src, local, tree, (r["x"], r["y"], r["theta"]),
                    max_iter=50, max_corr=fine_corr)
        refined.append(rr)
    if not refined:
        raise ValueError("双阶段 ICP 无有效候选")

    max_fit = max(r["fitness"] for r in refined)
    ok = [r for r in refined if r["fitness"] >= 0.7 * max_fit and r["rmse"] > 0]
    best = min(ok, key=lambda r: r["rmse"]) if ok else max(refined, key=lambda r: r["fitness"])
    best = dict(best)
    best["n_starts"] = len(positions) * len(yaws)
    best["coarse_std"] = _pose_spread(refined)
    return best


# ----------------------------------------------------------------------------
# 无初值全局匹配:似然场(EDT) 多分辨率 相关性搜索 -> ICP 精修
# ----------------------------------------------------------------------------
def _build_likelihood(mp, occ_thresh=OCC_THRESH):
    """障碍二值图的欧氏距离变换(米):free 格到最近障碍的距离。"""
    grid, meta = _occ_grid(mp)
    obstacle = grid >= occ_thresh
    if not obstacle.any():
        raise ValueError("地图没有障碍点,无法全局匹配")
    edt_cells = distance_transform_edt(~obstacle)   # 每个非障碍格到最近障碍(格)
    edt_m = edt_cells.astype(np.float64) * meta["res"]  # -> 米;障碍格本身为 0
    return edt_m, meta, obstacle


def _score_pose(src, edt_m, meta, x, y, theta, inlier_thresh=0.3):
    """把 src 变换到世界系,查 EDT:返回 (内点数, 内点均距)。分越高越贴合。"""
    cur = _transform(src, x, y, theta)
    col, row, valid = _world_to_cell(cur, meta)
    if not valid.any():
        return 0, float("inf")
    dvals = np.full(len(cur), np.inf)
    dvals[valid] = edt_m[row[valid], col[valid]]
    inl = dvals < inlier_thresh
    n = int(inl.sum())
    mean_d = float(dvals[inl].mean()) if n else float("inf")
    return n, mean_d


def global_match(src, mp, map_pts=None,
                 coarse_step=0.5, coarse_yaw_bins=16, inlier_thresh=0.35,
                 refine_step=0.15, refine_yaw_bins=12, refine_span=1.0,
                 keep=5, max_scan=250):
    """无初值全局配准:EDT 相关性搜索(粗->细)-> 2D ICP 精修。"""
    edt_m, meta, obstacle = _build_likelihood(mp)
    if map_pts is None:
        map_pts = occupancy_to_points(mp)
    tree = cKDTree(map_pts)

    # 全局搜索用抽稀的 scan,加快每个位姿打分
    s = src
    if len(s) > max_scan:
        sel = np.linspace(0, len(s) - 1, max_scan).astype(int)
        s = s[sel]

    # 搜索范围 = 障碍点包围盒(机器人必在结构附近)
    lo = map_pts.min(axis=0)
    hi = map_pts.max(axis=0)
    xs = np.arange(lo[0], hi[0] + coarse_step, coarse_step)
    ys = np.arange(lo[1], hi[1] + coarse_step, coarse_step)
    yaws = [2.0 * math.pi * k / coarse_yaw_bins for k in range(coarse_yaw_bins)]

    cands = []  # (score, mean_d, x, y, theta)
    for x in xs:
        for y in ys:
            best_here = None
            for yaw in yaws:
                n, md = _score_pose(s, edt_m, meta, x, y, yaw, inlier_thresh)
                if best_here is None or n > best_here[0] or (n == best_here[0] and md < best_here[1]):
                    best_here = (n, md, x, y, yaw)
            if best_here and best_here[0] > 0:
                cands.append(best_here)
    if not cands:
        raise ValueError("全局搜索无任何候选命中,请确认地图/扫描是否匹配")
    cands.sort(key=lambda c: (-c[0], c[1]))
    top = cands[:keep]

    # 细化:在每个粗候选附近做更细的平移/旋转搜索
    refined = []
    r_yaws_off = [(-0.5 + k / (refine_yaw_bins - 1)) * (2.0 * math.pi / coarse_yaw_bins) * 2.0
                  for k in range(refine_yaw_bins)] if refine_yaw_bins > 1 else [0.0]
    offs = np.arange(-refine_span, refine_span + refine_step, refine_step)
    for (_, _, cx, cy, cyaw) in top:
        best_here = None
        for dx in offs:
            for dy in offs:
                for dyaw in r_yaws_off:
                    x, y, yaw = cx + dx, cy + dy, cyaw + dyaw
                    n, md = _score_pose(s, edt_m, meta, x, y, yaw, inlier_thresh)
                    if best_here is None or n > best_here[0] or (n == best_here[0] and md < best_here[1]):
                        best_here = (n, md, x, y, yaw)
        if best_here:
            refined.append(best_here)
    refined.sort(key=lambda c: (-c[0], c[1]))

    # 对细化 top 候选做 ICP 精修,取最优
    icp_results = []
    for (_, _, x, y, yaw) in refined[:keep]:
        rr = icp_2d(src, map_pts, tree, (x, y, yaw), max_iter=50, max_corr=0.3)
        icp_results.append(rr)
    if not icp_results:
        raise ValueError("全局匹配精修失败")
    max_fit = max(r["fitness"] for r in icp_results)
    ok = [r for r in icp_results if r["fitness"] >= 0.7 * max_fit and r["rmse"] > 0]
    best = min(ok, key=lambda r: r["rmse"]) if ok else max(icp_results, key=lambda r: r["fitness"])
    best = dict(best)
    best["n_starts"] = len(top)
    best["coarse_std"] = _pose_spread(icp_results)
    return best


def _pose_spread(results):
    """一组结果的散布:位置 RMS(米) + yaw 圆标准差(度)。散布小 = 高置信。"""
    good = [r for r in results if math.isfinite(r.get("rmse", float("inf")))]
    if len(good) < 2:
        return None
    P = np.array([[r["x"], r["y"]] for r in good], dtype=np.float64)
    c = P.mean(axis=0)
    pos_rms = float(math.sqrt(np.mean(np.sum((P - c) ** 2, axis=1))))
    angs = np.array([r["theta"] for r in good], dtype=np.float64)
    C, S = np.mean(np.cos(angs)), np.mean(np.sin(angs))
    R = float(np.clip(math.hypot(C, S), 1e-9, 1.0))
    yaw_std_deg = float(math.degrees(math.sqrt(-2.0 * math.log(R))))
    return {"n": len(good), "pos_rms": round(pos_rms, 4), "yaw_std_deg": round(yaw_std_deg, 2)}


# ----------------------------------------------------------------------------
# 顶层入口
# ----------------------------------------------------------------------------
def relocalize(method, scan, mp, init=None, params=None, rmse_accept=0.15):
    """统一入口。
    method: 'global' | 'two_stage'(init 必给位置)
    scan/mp: on_scan / on_map 的 payload
    init: {'x':..,'y':..,'yaw':可选}  (two_stage 用)
    返回 dict:pose(x,y,yaw)/rmse/fitness/accepted/method/... ;失败抛 ValueError。
    """
    params = params or {}
    src = scan_to_points(scan)
    if len(src) < 30:
        raise ValueError("当前雷达扫描点太少,请确认激光在发数据")
    map_pts = occupancy_to_points(mp, occ_thresh=params.get("occ_thresh", OCC_THRESH))
    if len(map_pts) < 50:
        raise ValueError("地图障碍点过少,请确认已加载有效地图")

    if method == "global":
        res = global_match(src, mp, map_pts=map_pts,
                            coarse_step=params.get("coarse_step", 0.5),
                            coarse_yaw_bins=params.get("coarse_yaw_bins", 16),
                            inlier_thresh=params.get("inlier_thresh", 0.35))
    elif method == "two_stage":
        if not init or init.get("x") is None or init.get("y") is None:
            raise ValueError("two_stage 需要初始位置 init={x,y}")
        res = two_stage_icp(src, map_pts, (float(init["x"]), float(init["y"])),
                            init_yaw=init.get("yaw"),
                            crop_r=params.get("crop_r", 8.0),
                            ring_r=params.get("ring_r", 0.5),
                            n_ring=params.get("n_ring", 6),
                            n_yaw=params.get("n_yaw", 12))
    else:
        raise ValueError(f"未知 method: {method}")

    yaw = _wrap(res["theta"])
    return {
        "method": method,
        "pose": {"x": round(res["x"], 4), "y": round(res["y"], 4),
                 "yaw": round(yaw, 6), "yaw_deg": round(math.degrees(yaw), 2)},
        "rmse": round(res["rmse"], 4) if math.isfinite(res["rmse"]) else None,
        "fitness": round(res["fitness"], 4),
        "inliers": res.get("inliers"),
        "n_starts": res.get("n_starts"),
        "spread": res.get("coarse_std"),
        "accepted": bool(math.isfinite(res["rmse"]) and res["rmse"] <= rmse_accept),
        "n_scan": int(len(src)),
        "n_map": int(len(map_pts)),
    }


# ----------------------------------------------------------------------------
# 合成数据自测:python -m backend.app.reloc2d.matcher
# ----------------------------------------------------------------------------
def _selftest():
    import time
    rng = np.random.default_rng(0)
    res = 0.05
    w = h = 400  # 20m x 20m
    grid = np.zeros((h, w), dtype=np.int16)
    # 造一个矩形房间墙(障碍)
    grid[50, 50:350] = 100
    grid[350, 50:350] = 100
    grid[50:350, 50] = 100
    grid[50:350, 350] = 100
    grid[200, 50:200] = 100  # 一道内墙,破坏对称性
    mp = {"resolution": res, "width": w, "height": h,
          "origin": {"x": -10.0, "y": -10.0, "z": 0.0, "yaw": 0.0},
          "data": grid.reshape(-1).tolist()}

    # 真值位姿
    gt = (1.3, -0.7, math.radians(40))
    map_pts = occupancy_to_points(mp)
    # 用真值把地图障碍点“逆变换”回激光系,当作理想 scan(加噪 + 抽稀模拟单线激光)
    Rt = _rot(-gt[2])
    laser = (map_pts - np.array([gt[0], gt[1]])) @ Rt.T
    # 只取一定范围内的点,模拟激光量程,并抽稀 + 噪声
    dd = np.linalg.norm(laser, axis=1)
    laser = laser[dd < 12.0]
    sel = np.linspace(0, len(laser) - 1, min(360, len(laser))).astype(int)
    laser = laser[sel] + rng.normal(0, 0.01, size=(len(sel), 2))
    # 构造 scan payload(把点还原成 ranges/angle)
    ang = np.arctan2(laser[:, 1], laser[:, 0])
    rng_r = np.linalg.norm(laser, axis=1)
    order = np.argsort(ang)
    scan = {"angle_min": float(ang[order][0]), "angle_increment": 0.0,
            "ranges": None}
    # 用规则角度栅格重建(更贴近真实 LaserScan)
    a0, a1 = -math.pi, math.pi
    n = 720
    inc = (a1 - a0) / n
    ranges = [None] * n
    for a, r in zip(ang, rng_r):
        i = int((a - a0) / inc)
        if 0 <= i < n:
            if ranges[i] is None or r < ranges[i]:
                ranges[i] = float(r)
    scan = {"angle_min": a0, "angle_increment": inc, "ranges": ranges}

    print(f"真值 pose = x={gt[0]:.3f} y={gt[1]:.3f} yaw={math.degrees(gt[2]):.1f}deg")

    t0 = time.time()
    r2 = relocalize("two_stage", scan, mp, init={"x": 1.0, "y": -1.0})
    print(f"[two_stage] {r2['pose']} rmse={r2['rmse']} fit={r2['fitness']} "
          f"accepted={r2['accepted']} ({(time.time()-t0)*1000:.0f}ms)")

    t0 = time.time()
    r1 = relocalize("global", scan, mp)
    print(f"[global]    {r1['pose']} rmse={r1['rmse']} fit={r1['fitness']} "
          f"accepted={r1['accepted']} ({(time.time()-t0)*1000:.0f}ms)")


if __name__ == "__main__":
    _selftest()
