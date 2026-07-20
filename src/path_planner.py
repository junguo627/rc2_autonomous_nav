"""
路径规划模块 — 红蓝双方共用。
给定队伍颜色和地形配置，返回结构化指令序列供 auto_7.13.3.py 执行。

用法:
    from path_planner import plan_route

    plan = plan_route("红方", RED_CONFIG)
    # plan["pre_cmds"]   → 底部前置拾取指令
    # plan["main_cmds"]  → 主路径指令序列
    # plan["exit_pos"]   → (实际行, 实际列)
    # plan["total_picked"] → 拾取总数

每条指令格式: ("函数名", (位置参数,), {关键字参数})
"""

import heapq

# =============================
# 红蓝双方差异配置
# =============================
TEAM_CONFIGS = {
    "红方": {
        "height": [
            [-1, 0, 0, 0, -1],  # 行1
            [-1, 1, 2, 1, -1],  # 行2
            [-1, 2, 3, 2, -1],  # 行3
            [-1, 3, 2, 1, -1],  # 行4
            [-1, 2, 1, 2, -1],  # 行5
            [-1, 0, 0, 0, -1],  # 行6
        ],
        # 出口优先级: (行1列4) 优先, 次选 (行1列2)
        "exits": [(0, 3), (0, 1)],
        # 底部平路移动坐标: (target_x, target_y, target_yaw)
        "bottom_pos": {
            2: (2.442, -0.439, 0.0),   # (6,2)
            3: (2.442, -1.606, 0.0),   # (6,3) 初始位
            4: (2.442, -2.941, 0.0),   # (6,4)
        },
    },
    "蓝方": {
        "height": [
            [-1, 0, 0, 0, -1],  # 行1
            [-1, 1, 2, 1, -1],  # 行2
            [-1, 2, 3, 2, -1],  # 行3
            [-1, 1, 2, 3, -1],  # 行4（蓝方核心差异：左低右高）
            [-1, 2, 1, 2, -1],  # 行5
            [-1, 0, 0, 0, -1],  # 行6
        ],
        # 出口优先级: (行1列2) 优先, 次选 (行1列4)
        "exits": [(0, 1), (0, 3)],
        # 底部平路移动坐标: (target_x, target_y, target_yaw)
        "bottom_pos": {
            2: (2.432, 2.758, 0.0),   # (6,2)  ← 与红方不同时改这里
            3: (2.432, 1.551, 0.0),   # (6,3)  ← 与红方不同时改这里
            4: (2.432, 0.350, 0.0),   # (6,4)  ← 与红方不同时改这里
        },
    },
}

# 方向定义：行偏移、列偏移、方向名、朝向yaw
DIRS = [
    (-1, 0, 'U', 0),
    (1,  0, 'D', 180),
    (0, -1, 'L', 90),
    (0,  1, 'R', 270),
]


# ===================== 指令构造工具 =====================
def _cmd(func_name, *args, **kwargs):
    """构造一条结构化指令: ("func_name", (args,), {kwargs})"""
    return (func_name, args, kwargs)


# ===================== 核心规则函数 =====================
def get_move_yaw(direction, dh):
    """根据移动方向和高度差，返回移动所需的朝向 yaw"""
    if dh == 1:  # 上台阶: 朝向与移动方向一致
        yaw_map = {'U': 0, 'D': 180, 'L': 90, 'R': 270}
        return yaw_map[direction]
    elif dh == -1:  # 下台阶: 朝向与移动方向相反
        yaw_map = {'U': 180, 'D': 0, 'L': 270, 'R': 90}
        return yaw_map[direction]
    return 0


def get_pick_cmd(dh, total_carried):
    """根据已携带总数选择拾取指令
    0个 → 1/2 拾取到手上
    1个 → 4/5 拾取并存入装置
    """
    if total_carried == 0:
        return 1 if dh == 1 else 2
    elif total_carried == 1:
        return 4 if dh == 1 else 5
    return None


# ===================== 地形配置与校验 =====================
def init_status_matrix():
    """初始化 6x5 状态矩阵，可配置区域(行2~5,列2~4)标记为'?'"""
    status = [['n' for _ in range(5)] for _ in range(6)]
    for r in range(1, 5):
        for c in range(1, 4):
            status[r][c] = '?'
    return status


def apply_config(status, config):
    """将 user_config 应用到状态矩阵"""
    for (real_row, real_col), val in config.items():
        r, c = real_row - 1, real_col - 1
        if not (1 <= r <= 4 and 1 <= c <= 3):
            raise ValueError(f"位置({real_row},{real_col})不属于可配置区域")
        if val not in ('t', 'f', 'n'):
            raise ValueError(f"无效值 {val}，仅支持 t/f/n")
        status[r][c] = val
    return status


def validate_config(status, team_label=""):
    """校验地形配置的合法性"""
    prefix = f"[{team_label}] " if team_label else ""
    t_cnt = sum(row.count('t') for row in status)
    f_cnt = sum(row.count('f') for row in status)
    if t_cnt < 2 or t_cnt > 4:
        raise ValueError(f"{prefix}目标t数量需要2~4个，当前为{t_cnt}")
    if f_cnt != 1:
        raise ValueError(f"{prefix}障碍f数量必须为1，当前为{f_cnt}")
    if status[4][2] == 'f':
        raise ValueError(f"{prefix}入口位置(5,3)不能设置为障碍f")
    return True


def print_terrain(height, status, team_label=""):
    """打印地形可视化（调试用）"""
    label = f"{team_label}" if team_label else "当前"
    print(f"=== {label}地形 ===")
    for r in range(6):
        row_items = []
        for c in range(5):
            row_items.append(f"({height[r][c]},{status[r][c]})")
        print("[" + ", ".join(f"{item:>6}" for item in row_items) + "]")
    print()


# ===================== 底部前置 t 处理 =====================
def preprocess_bottom_t(status, bottom_pos):
    """处理第5行左右两侧的 t，生成前置指令和初始状态
    触发条件：(5,2)或(5,4)有t，且(5,3)无t
    """
    pre_cmds = []
    init_storage = 0
    init_picked = set()

    has_t_left = status[4][1] == 't'
    has_t_right = status[4][3] == 't'
    has_t_mid = status[4][2] == 't'

    if (has_t_left or has_t_right) and not has_t_mid:
        # ★ 左右都有 t 时只拿一个（靠出口侧优先），剩余由主 Dijkstra 处理
        pick_left = has_t_left and not has_t_right   # 仅左有 → 拿左
        pick_right = has_t_right and not has_t_left  # 仅右有 → 拿右
        if has_t_left and has_t_right:
            pick_left = True   # 两侧都有时优先拿左（蓝方出口在左），另一个留给主规划
            pick_right = False

        if pick_left:
            x, y, yaw = bottom_pos[2]
            pre_cmds.append(_cmd("auto_go_to_target", target_x=x, target_y=y, target_yaw=yaw))
            pre_cmds.append(_cmd("run_getkfs", 6))
            init_storage += 1
            init_picked.add((4, 1))
        if pick_right:
            x, y, yaw = bottom_pos[4]
            pre_cmds.append(_cmd("auto_go_to_target", target_x=x, target_y=y, target_yaw=yaw))
            pre_cmds.append(_cmd("run_getkfs", 6))
            init_storage += 1
            init_picked.add((4, 3))
        x, y, yaw = bottom_pos[3]
        pre_cmds.append(_cmd("auto_go_to_target", target_x=x, target_y=y, target_yaw=yaw))

    return pre_cmds, init_storage, frozenset(init_picked)


# ===================== 路径规划核心（Dijkstra） =====================
def plan_path(status, init_storage, init_picked, height, exits):
    """基于 Dijkstra 搜索最优路径"""
    start_r, start_c = 5, 2   # (6,3)
    start_yaw = 0
    max_carry = 2

    # 出口距离惩罚权重: 拾取目标后携带距离越远跌落风险越大，偏好离出口近的目标
    EXIT_DIST_WEIGHT = 5

    # 优先队列: (-拾取总数, cost, 指令数, r, c, yaw, 手上持有, 装置存放, 已拾取集合, 指令列表)
    # cost = 移动步数 + 拾取时离出口距离惩罚（仅当手上有货时）
    heap = []
    initial = (
        -(0 + init_storage), 0, 0,
        start_r, start_c, start_yaw,
        0, init_storage, init_picked, []
    )
    heapq.heappush(heap, initial)

    visited = set()
    visited.add((start_r, start_c, start_yaw, 0, init_storage, init_picked))

    while heap:
        neg_total, steps, cmd_cnt, r, c, yaw, hand, storage, picked, cmds = heapq.heappop(heap)
        total = -neg_total

        # 到达出口时返回
        if (r, c) in exits and total == min(max_carry, sum(row.count('t') for row in status)):
            return cmds, (r + 1, c + 1), total
        if (r, c) in exits and total == max_carry:
            return cmds, (r + 1, c + 1), total

        # ---- 1. 原地拾取 ----
        if total < max_carry:
            for dr, dc, dir_name, pick_yaw in DIRS:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < 6 and 0 <= nc < 5):
                    continue
                if height[nr][nc] == -1:
                    continue
                if status[nr][nc] != 't' or (nr, nc) in picked:
                    continue

                dh = height[nr][nc] - height[r][c]
                if abs(dh) != 1:
                    continue

                new_cmds = cmds.copy()
                cur_yaw = yaw
                if cur_yaw != pick_yaw:
                    new_cmds.append(_cmd("run_rotate_stage", pick_yaw))
                    cur_yaw = pick_yaw

                pick_cmd_val = get_pick_cmd(dh, total)
                new_cmds.append(_cmd("run_getkfs", pick_cmd_val))
                if pick_cmd_val == 4:
                    new_cmds.append(_cmd("sleep_check_retry", 3))

                new_picked = picked | frozenset([(nr, nc)])
                if total == 0:
                    new_hand = 1
                    new_storage = storage
                else:
                    new_hand = hand
                    new_storage = storage + 1
                new_total = new_hand + new_storage

                # ★ 手上已有货时，给离出口远的拾取点加惩罚
                pick_penalty = 0
                if new_total >= 1:
                    min_exit_dist = min(abs(nr - er) + abs(nc - ec) for er, ec in exits)
                    pick_penalty = min_exit_dist * EXIT_DIST_WEIGHT

                state = (r, c, cur_yaw, new_hand, new_storage, new_picked)
                if state not in visited:
                    visited.add(state)
                    heapq.heappush(heap, (
                        -new_total, steps + pick_penalty, cmd_cnt + len(new_cmds) - len(cmds),
                        r, c, cur_yaw, new_hand, new_storage, new_picked, new_cmds
                    ))

        # ---- 2. 原地丢出（移开挡路t） ----
        if hand > 0:
            for dr, dc, dir_name, drop_yaw in DIRS:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < 6 and 0 <= nc < 5):
                    continue
                if height[nr][nc] == -1:
                    continue
                if status[nr][nc] == 'f':
                    continue

                dh = height[nr][nc] - height[r][c]
                if abs(dh) != 1:
                    continue

                new_cmds = cmds.copy()
                cur_yaw = yaw
                if cur_yaw != drop_yaw:
                    new_cmds.append(_cmd("run_rotate_stage", drop_yaw))
                    cur_yaw = drop_yaw

                new_cmds.append(_cmd("run_getkfs", 7 if dh == 1 else 8))

                new_hand = hand - 1
                new_storage = storage
                new_total = new_hand + new_storage

                state = (r, c, cur_yaw, new_hand, new_storage, picked)
                if state not in visited:
                    visited.add(state)
                    heapq.heappush(heap, (
                        -new_total, steps, cmd_cnt + len(new_cmds) - len(cmds),
                        r, c, cur_yaw, new_hand, new_storage, picked, new_cmds
                    ))

        # ---- 3. 移动 ----
        for dr, dc, dir_name, _ in DIRS:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < 6 and 0 <= nc < 5):
                continue
            if height[nr][nc] == -1:
                continue
            if status[nr][nc] == 'f':
                continue
            if status[nr][nc] == 't' and (nr, nc) not in picked:
                continue

            # 入口限制：仅能从(5,3)进入地形主体
            if dir_name == 'U' and r == 5 and nr == 4 and nc != 2:
                continue
            # 出口限制：行1仅允许列2、4通行
            if dir_name == 'U' and r == 1 and nr == 0 and nc not in (1, 3):
                continue

            dh = height[nr][nc] - height[r][c]
            if abs(dh) != 1 and dh != 0:
                continue

            move_yaw = get_move_yaw(dir_name, dh)
            new_cmds = cmds.copy()
            cur_yaw = yaw
            # ★ 上/下台阶前始终微调角度，保证稳定性
            if dh != 0:
                new_cmds.append(_cmd("run_rotate_stage", move_yaw))
                cur_yaw = move_yaw
            elif cur_yaw != move_yaw:
                new_cmds.append(_cmd("run_rotate_stage", move_yaw))
                cur_yaw = move_yaw

            if dh == 1:
                new_cmds.append(_cmd("set_step_mode", 1))
            elif dh == -1:
                new_cmds.append(_cmd("set_step_mode", 2))

            state = (nr, nc, cur_yaw, hand, storage, picked)
            if state not in visited:
                visited.add(state)
                heapq.heappush(heap, (
                    -total, steps + 1, cmd_cnt + len(new_cmds) - len(cmds),
                    nr, nc, cur_yaw, hand, storage, picked, new_cmds
                ))

    return None, None, 0


# ===================== 公开入口 =====================
def plan_route(team, user_config, verbose=True):
    """路径规划主入口。

    参数:
        team:        "红方" 或 "蓝方"
        user_config: {(实际行, 实际列): 't'/'f'/'n', ...}
                     可配置区域: 行2~5, 列2~4
        verbose:     是否打印地形和规划摘要

    返回:
        dict: {
            "pre_cmds":      [(func_name, args_tuple, kwargs_dict), ...],
            "main_cmds":     [(func_name, args_tuple, kwargs_dict), ...],
            "exit_pos":      (实际行, 实际列),
            "total_picked":  int,
        }
        规划失败时返回 None
    """
    if team not in TEAM_CONFIGS:
        raise ValueError(f"无效队伍: {team}，仅支持 '红方' 或 '蓝方'")

    cfg = TEAM_CONFIGS[team]
    height = cfg["height"]
    exits = cfg["exits"]
    bottom_pos = cfg["bottom_pos"]

    # 1. 校验 & 构建状态矩阵
    status = init_status_matrix()
    status = apply_config(status, user_config)
    validate_config(status, team_label=team)

    if verbose:
        print_terrain(height, status, team_label=team)

    # 2. 底部前置处理
    pre_cmds, init_storage, init_picked = preprocess_bottom_t(status, bottom_pos)
    if verbose and pre_cmds:
        print("=== 底部前置拾取指令 ===")
        for cmd in pre_cmds:
            _print_cmd(cmd)
        print()

    # 3. 主路径规划
    main_cmds, exit_pos, total_picked = plan_path(
        status, init_storage, init_picked, height, exits
    )

    if main_cmds is None:
        if verbose:
            print("未找到可行路径，请调整障碍或目标位置。")
        return None

    if verbose:
        print(f"规划完成，出口位置：{exit_pos}，共拾取目标数：{total_picked}")
        print("=== 主路径指令序列 ===")
        for cmd in main_cmds:
            _print_cmd(cmd)

    return {
        "pre_cmds": pre_cmds,
        "main_cmds": main_cmds,
        "exit_pos": exit_pos,
        "total_picked": total_picked,
    }


def _print_cmd(cmd):
    """调试用：将结构化指令还原为人类可读字符串"""
    func_name, args, kwargs = cmd
    parts = []
    for a in args:
        parts.append(repr(a))
    for k, v in kwargs.items():
        parts.append(f"{k}={v!r}")
    print(f"  {func_name}({', '.join(parts)})")


# ===================== 命令行入口（兼容旧用法） =====================
if __name__ == "__main__":
    import sys

    # 用法: python path_planner.py 红方  或  python path_planner.py 蓝方
    team = sys.argv[1] if len(sys.argv) > 1 else "红方"

    if team == "红方":
        user_config = {
            (2, 2): 't', (2, 3): 'f', (2, 4): 'n',
            (3, 2): 'n', (3, 3): 't', (3, 4): 'n',
            (4, 2): 'n', (4, 3): 't', (4, 4): 'n',
            (5, 2): 'n', (5, 3): 'n', (5, 4): 't',
        }
    else:
        user_config = {
            (2, 2): 'n', (2, 3): 't', (2, 4): 'n',
            (3, 2): 'n', (3, 3): 't', (3, 4): 'n',
            (4, 2): 'n', (4, 3): 't', (4, 4): 'f',
            (5, 2): 'n', (5, 3): 't', (5, 4): 'n',
        }

    try:
        plan_route(team, user_config, verbose=True)
    except ValueError as e:
        print(f"配置错误：{e}")
