# 加了getKFS的几种方式
#加了重试
#retry也分红蓝方
#加0x07
#改进重试，想适用于梅林单项，也想加摄像头函数
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from auto_aim_interfaces.msg import Fas, Target
import argparse
import math
import serial
import time
import struct
import threading
import subprocess
import signal
import os
import glob
from path_planner import plan_route
from terrain_editor import run_terrain_editor, load_config_from_file

# ===================== 基础配置 =====================
# 串口自动识别: 启动时扫描 /dev/ttyACM* 和 /dev/ttyUSB*，运行时断开也会自动重连
SERIAL_PORT = None  # None=自动扫描，也可手动指定如 "/dev/ttyACM0"
BAUD_RATE = 921600
SEND_FREQ = 1000
DT = 1.0 / SEND_FREQ

# 速度上限
MAX_VX = 0.5
MAX_VY = 0.5
MAX_W = 0.5

# 底盘PID控制参数
KP_X = 1.0
KP_Y = 1.0
KP_YAW = 1.2
POS_TOL = 0.2          # 位置到达阈值 (m)
YAW_TOL = 0.5           # 角度到达阈值（度）

# ===================== 里程计健康监控 =====================
ODOM_MAX_INTERVAL = 0.1      # 里程计消息最大间隔(秒)，超过则重启雷达+里程计
ODOM_RESTART_COOLDOWN = 5.0  # 两次重启之间的最小间隔(秒)，防止反复重启

# ===================== 到达判断 / 超时 / 重试参数 =====================
DWELL_CYCLES = 30        # 到达容差后需连续停留的周期数 (0.3s@100Hz)，防止噪声误判
STOP_DWELL = 15          # 阶段完成后持续发送零速的周期数 (0.15s)，确保真停稳
MAX_RETRIES = 3          # 每阶段最大重试次数
MAX_STAGE_CYCLES = 3600  # 每阶段超时周期数 (~60秒)，防止定位丢失后无限行驶

# ===================== 视觉对准参数 =====================
MODEL_PATH = "/home/ubuntu-nuc/yolo_new/v4.pt"
CAMERA_ID = 1
KP_PIXEL = 0.0015         # 像素误差→vx (0.5/320≈0.0015)
PIXEL_TOL = 40            # 像素容差(px) — 放宽避免来回振荡
VISUAL_MAX_VX = 0.2       # 视觉对准最大速度
VISUAL_MIN_VX = 0.05      # 视觉对准最小速度（克服静摩擦，降低以减少过冲）
VISUAL_DWELL = 30         # 对准驻留确认帧数
VISUAL_TIMEOUT = 10.0     # 视觉对准超时(秒)
VISUAL_NO_DET_VX = -0.2   # 无检测目标时左移搜索


# ===================== YOLO 视觉识别函数 =====================
# 串口接收帧 (下位机→上位机) — 新协议 ComputerTransmit_Frame_S
# typedef struct __attribute__((packed)) {
#     uint8_t header;                  // 帧头
#     uint8_t Calibration_flag;        // 校准标志位, 0x00=未校准, 0x01=已校准
#     uint8_t Lift_flag;               // 升降状态, 0=未升降, 1=上台阶, 2=下台阶
#     uint8_t GetWeapon_FinshFlag;     // 拾取武器完成标志位
#     uint8_t GetKFS_Flag;             // KFS状态
#     float   Eul_YAW;                 // 当前yaw角
#     uint8_t tail;                    // 帧尾
# } ComputerTransmit_Frame_S;
RECV_HEADER = 0xAA
RECV_TAIL = 0x0D
RECV_FRAME_LEN = 11  # 新协议: header(1)+Calib(1)+Lift(1)+Weapon(1)+KFS(1)+Yaw(4)+tail(1)=10
CALIB_DONE_MASK = 0x01   # 接收帧 byte[1] 的校准完成标志位

# 新协议发送帧 (上位机→下位机, Computer_Frame_S packed)
# typedef struct __attribute__((packed)) {
#     uint8_t header;               // 帧头
#     uint8_t Calibration_Flag;     // 校准标志位, 0=未请求, 1=请求
#     float   cmd_yaw;              // 航向角指令(度), [0,360)
#     uint8_t cmd_lift;             // 升降指令
#     float   GetWeapon_StartFlag;  // 拾取武器标志位, 置1=开始拾取
#     uint8_t AimtoGetKFSFlag;      // 瞄准拾取KFS标志位, 0x01=开始瞄准拾取 (新增)
#     uint8_t GetKFS_CMD;           // 吸盘控制指令, 0x00=不拾取, 0x01=向前拾取, 0x02=向下拾取, 0x03=向外侧放置KFS, 0x04=向前吸取并存放KFS, 0x05=向下吸取并存放KFS, 0x06=吸取高位KFS
#     float   Position_MeasureX;    // 里程计当前X
#     float   Position_MeasureY;    // 里程计当前Y
#     float   Position_Target_X;    // 里程计目标X
#     float   Position_Target_Y;    // 里程计目标Y
#     uint8_t tail;                 // 帧尾
# } Computer_Frame_S;
SEND_HEADER = 0xAA
SEND_TAIL = 0x0D
SEND_FRAME_LEN = 30  # 1+1+4+1+4+1+1+4+4+4+4+1 = 30

# ===================== 下位机协议全局控制变量 (Computer_Frame_S 字段) =====================
calib_switch = 1              # Calibration_Flag: 0=未请求校准, 1=请求校准
cmd_lift = 0                  # 升降指令 (uint8)
getweapon_startflag = 0.0     # GetWeapon_StartFlag: 拾取武器标志位, 置1表示开始拾取
AimtoGetKFSFlag = 0x00        # AimtoGetKFSFlag: 瞄准拾取KFS标志位, 0x01=开始瞄准拾取 (新增, 先设0)
GetKFS_CMD = 0x00             # GetKFS_CMD: 吸盘控制指令, 0x00=不拾取, 0x01=向前拾取, 0x02=向下拾取, 0x03=向外侧放置KFS, 0x04=向前吸取并存放KFS, 0x05=向下吸取并存放KFS, 0x06=吸取高位KFS
position_mode_flag = 0.0      # 位置模式标志: 0=不使用, 1=使用 (逻辑判断用, 不发送)

# 兼容旧接口的内部变量 (不在新协议帧中)
step_mode = 0
gripper_state = 0x00
suction_cup_control = 0x00    # 吸盘控制指令 (保留兼容)

# 全局底盘定位
ser = None
ser_lock = None
current_x = 0.0
current_y = 0.0
current_yaw = 0.0           # 从串口读取，单位：度
calib_done = False          # 下位机上报的校准完成标志
step_mode_feedback = 0      # 下位机上报的台阶状态: 0=空闲, 1=正在上台阶, 2=正在下台阶
lio_process = None
livox_process = None
shutdown_requested = False    # 全局退出标志，通知所有后台线程停止

# 里程计健康监控状态
odom_last_time = 0.0          # 上次 odom 回调时间戳
odom_max_interval = 0.0       # 当前窗口内最大消息间隔
odom_interval_exceeded = False  # 标记间隔是否超阈值
last_odom_restart_time = 0.0  # 上次重启时间 (用于冷却)

# 武器拿取状态 (getweapon_finishflag 由串口接收线程更新)
getweapon_finishflag = 0      # 拾取武器完成标志
getkfs_flag = 0x00             # KFS状态标志 (接收自下位机)
invert_coordinates = False    # 坐标取反标志: True时串口发送和到达判断都用取反后的xy

# ===================== 地形配置（路径规划用） =====================
# 地形配置 JSON 文件路径 — UI 编辑器读写此文件
TERRAIN_CONFIG_DIR = "/home/ubuntu-nuc/rc2_ws/src"

# 默认配置仅在 JSON 文件不存在且跳过 UI 时作为 fallback
# 红方默认: t=目标KFS, f=障碍, n=空地 (行2~5可配置)
RED_CONFIG_DEFAULT = {
    (2, 2): 't', (2, 3): 'f', (2, 4): 'n',
    (3, 2): 'n', (3, 3): 't', (3, 4): 'n',
    (4, 2): 'n', (4, 3): 't', (4, 4): 'n',
    (5, 2): 'n', (5, 3): 'n', (5, 4): 't',
}

# 蓝方默认: t=目标KFS, f=障碍, n=空地 (行2~5可配置)
BLUE_CONFIG_DEFAULT = {
    (2, 2): 'n', (2, 3): 't', (2, 4): 'n',
    (3, 2): 'n', (3, 3): 't', (3, 4): 'n',
    (4, 2): 'n', (4, 3): 't', (4, 4): 'f',
    (5, 2): 't', (5, 3): 'n', (5, 4): 'n',
}



# ★ 导航取消标志: 设为 True 可中断当前 auto_go_to_target / run_rotate_stage
cancel_nav = False

# ★ retry 重试机制: 串口收到 Retry_Flag ∈ {0x01,0x02,0x03,0x04,0x05} 时触发任务切换
retry_triggered = 0            # 0=正常, 非0=触发的 retry 值 (被 serial_read_loop 设置)
retry_disabled = False         # True=禁止串口线程再次触发 retry，防止 retry 任务中被同一 flag 反复中断
last_retry_flag = 0            # 上一次收到的 retry_flag，用于边沿检测（值变化时才触发）
RETRY_WAIT_SEC = 0.5           # 收到 retry 后等待秒数再执行新任务


# ===================== RetryInterrupt 异常 & 工具函数 =====================
class RetryInterrupt(Exception):
    """串口收到 Retry_Flag 时抛出，中断当前任务序列并切换到对应的 retry 任务"""
    def __init__(self, retry_val: int):
        super().__init__(f"RetryInterrupt: retry_flag=0x{retry_val:02X}")
        self.retry_val = retry_val


def check_retry():
    """所有阻塞函数循环体内调用: 若 retry_triggered 已触发则抛出 RetryInterrupt"""
    if retry_triggered:
        raise RetryInterrupt(retry_triggered)


def sleep_check_retry(seconds, interval=0.1):
    """可被 retry 中断的 sleep — 替代 mission_fn 中的 time.sleep()"""
    for _ in range(int(seconds / interval)):
        if retry_triggered:
            raise RetryInterrupt(retry_triggered)
        time.sleep(interval)

# ★ 持续发送线程的共享导航目标（速度模式）
g_target_vx = 0.0
g_target_vy = 0.0
g_target_yaw = None         # None=使用串口收到的 current_yaw

# ★ 位置模式共享目标（由 auto_go_to_target 写入，发送线程读取）
g_pos_target_x = 0.0
g_pos_target_y = 0.0
g_pos_target_yaw = None  # None=不控制yaw, 否则为角度值(deg)


# ===================== 串口工具函数 =====================
def find_serial_port():
    """自动扫描可用串口，返回设备路径或 None。
       扫描顺序: /dev/ttyACM* → /dev/ttyUSB*，取第一个找到的。
    """
    candidates = sorted(glob.glob('/dev/ttyACM*')) + sorted(glob.glob('/dev/ttyUSB*'))
    if candidates:
        print(f"[串口扫描] 发现可用串口: {candidates}")
        return candidates[0]
    return None


def init_serial(port=None):
    """初始化串口连接。port=None 时自动扫描；也可传入如 "/dev/ttyACM0" 指定。"""
    global ser, ser_lock, SERIAL_PORT
    # 先关闭旧连接
    try:
        if ser is not None and ser.is_open:
            ser.close()
            time.sleep(0.2)
    except:
        pass
    ser = None

    if port is None:
        port = find_serial_port()
    if port is None:
        print("[串口] 未发现可用串口设备 (ttyACM*/ttyUSB*)")
        return False

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.01)
        ser_lock = threading.Lock()
        SERIAL_PORT = port
        print(f"[串口] 连接成功 → {port}")
        return True
    except Exception as e:
        print(f"[串口] 连接失败 ({port}): {e}")
        return False


def ensure_serial_connected():
    """检查串口是否存活；若已断开则自动扫描并重连。
       返回 True 表示串口当前可用，False 表示暂时无法连接。
       特别处理 ACM0↔ACM1 互换：旧端口消失后自动扫描新端口。
    """
    global ser, ser_lock, SERIAL_PORT
    if ser is not None and ser.is_open:
        # 快速健康检查：读 in_waiting 属性，如果设备文件已消失会抛异常
        try:
            _ = ser.in_waiting
            return True
        except (serial.SerialException, OSError):
            print("[串口] 健康检查失败，设备已断开，准备重连...")
            try:
                ser.close()
            except:
                pass
            ser = None

    # 串口不可用 → 尝试重新扫描连接
    print("[串口] 尝试重新扫描并连接...")
    return init_serial(port=None)


def send_full_frame(vx: float, vy: float, yaw_abs: float):
    """按新 Computer_Frame_S 协议打包发送 30 字节帧."""
    if ser is None or not ser.is_open or shutdown_requested:
        return
    try:
        # ★ 坐标取反: 蓝方前两个点需要反转坐标系
        if invert_coordinates:
            tx = -current_x
            ty = -current_y
            ttx = -g_pos_target_x
            tty = -g_pos_target_y
        else:
            tx = current_x
            ty = current_y
            ttx = g_pos_target_x
            tty = g_pos_target_y

        full_frame = (
            struct.pack("<B", SEND_HEADER) +
            struct.pack("<B", calib_switch) +
            struct.pack("<f", yaw_abs) +
            struct.pack("<B", cmd_lift) +
            struct.pack("<f", getweapon_startflag) +
            struct.pack("<B", AimtoGetKFSFlag) +
            struct.pack("<B", GetKFS_CMD) +
            struct.pack("<f", tx) +
            struct.pack("<f", ty) +
            struct.pack("<f", ttx) +
            struct.pack("<f", tty) +
            struct.pack("<B", SEND_TAIL)
        )

        if len(full_frame) != SEND_FRAME_LEN:
            print(f"报文长度异常:{len(full_frame)} (期望{SEND_FRAME_LEN})")
            return

        with ser_lock:
            ser.write(full_frame)
        if not hasattr(send_full_frame, '_print_cnt'):
            send_full_frame._print_cnt = 0
        send_full_frame._print_cnt += 1



        if send_full_frame._print_cnt % 10 == 0:
            print(f"底盘 vx={vx:.3f},vy={vy:.3f},send_yaw={yaw_abs:.1f} deg | "
                  f"tx={tx:.3f},ty={ty:.3f},yaw={current_yaw:.1f} deg | "
                  f"ttxy=({ttx:.3f},{tty:.3f}) | "
                  f"in_waiting={ser.in_waiting} | "
                  f"cmd_lift={cmd_lift:<5.1f} |"
                  f"getweapon_startflag={getweapon_startflag:<5.1f} |")
    except Exception as e:
        print(f"发送报文异常:{e}")


def stop_car():
    """车辆急停，速度清零 + 位置模式回退（target=当前坐标，下位机不动）"""
    global g_target_vx, g_target_vy, g_target_yaw
    global position_mode_flag, g_pos_target_x, g_pos_target_y, g_pos_target_yaw
    g_target_vx = 0.0
    g_target_vy = 0.0
    g_target_yaw = None
    position_mode_flag = 0.0
    g_pos_target_x = current_x
    g_pos_target_y = current_y
    g_pos_target_yaw = None
    time.sleep(0.05)


def stop_auto_nav():
    """
    停止当前正在执行的 auto_go_to_target / run_rotate_stage。
    调用后阻塞中的导航函数会立即返回 False，任务序列可继续执行后续点。

    使用方式：在另一个线程中调用，或通过 Ctrl+C 触发后手动调用。
    """
    global cancel_nav, position_mode_flag, g_pos_target_x, g_pos_target_y, g_pos_target_yaw, g_target_yaw
    cancel_nav = True
    position_mode_flag = 0.0
    g_pos_target_x = current_x
    g_pos_target_y = current_y
    g_pos_target_yaw = None
    g_target_yaw = None
    print("\n[stop_auto_nav] ★ 导航已取消! 位置模式回退，后续点将继续执行")


# ===================== ★ 持续发送线程接口 =====================
def set_nav_target(vx, vy, yaw):
    """更新速度模式导航目标值（由导航/视觉函数调用）"""
    global g_target_vx, g_target_vy, g_target_yaw
    g_target_vx = vx
    g_target_vy = vy
    g_target_yaw = yaw


def reset_nav_target():
    """重置导航目标为 idle: vx=0, vy=0, yaw=跟随 current_yaw"""
    global g_target_vx, g_target_vy, g_target_yaw
    g_target_vx = 0.0
    g_target_vy = 0.0
    g_target_yaw = None


def set_yaw_target(yaw_deg):
    """只设置目标 yaw，vx/vy=0"""
    global g_target_vx, g_target_vy, g_target_yaw
    g_target_vx = 0.0
    g_target_vy = 0.0
    g_target_yaw = yaw_deg


# ===================== ★ 位置模式导航函数 (新) =====================
def auto_go_to_target(target_x=None, target_y=None, target_yaw=None, wait=True, timeout=240.0):
    """
    位置模式导航：设置目标坐标，由下位机做位置PID驱动底盘。
    同时可控制 yaw 角。

    参数:
        target_x, target_y: 目标坐标 (lidar 世界坐标系)
        target_yaw: 目标航向角 (deg), None=不控制yaw
        wait: True=阻塞等待到达, False=只设置目标立刻返回
        timeout: 到达超时时间(秒)

    返回: True=到达目标, False=超时/失败
    """
    global cancel_nav, position_mode_flag, g_pos_target_x, g_pos_target_y, g_pos_target_yaw, g_target_yaw

    if target_x is not None and target_y is not None:
        g_pos_target_x = float(target_x)
        g_pos_target_y = float(target_y)

        # ★ yaw 控制
        if target_yaw is not None:
            g_pos_target_yaw = float(target_yaw)
            g_target_yaw = g_pos_target_yaw
        else:
            g_pos_target_yaw = None
            g_target_yaw = None

        position_mode_flag = 1.0

        print(f"  目标: ({g_pos_target_x:.3f}, {g_pos_target_y:.3f})"
              + (f", yaw={g_pos_target_yaw:.1f} deg" if g_pos_target_yaw is not None else ""))
        print(f"  当前: ({current_x:.3f}, {current_y:.3f}), yaw={current_yaw:.1f} deg")
        dx = g_pos_target_x - current_x
        dy = g_pos_target_y - current_y
        ew = normalize_angle((g_pos_target_yaw if g_pos_target_yaw is not None else current_yaw) - current_yaw)
        print(f"  误差: dx={dx:+.3f}, dy={dy:+.3f}, dist={math.sqrt(dx*dx+dy*dy):.3f}"
              + (f", yaw_err={ew:+.1f} deg" if g_pos_target_yaw is not None else ""))

        if not wait:
            return True  # 非阻塞模式，设置完立刻返回

        # ★ 阻塞等待到达目标
        cancel_nav = False  # ★ 每次调用前重置取消标志
        dwell_count = 0
        t0 = time.time()
        while time.time() - t0 < timeout:
            check_retry()
            if cancel_nav:
                print(f"  [auto_go_to_target] ★ 收到取消信号，中止导航")
                return False
            if shutdown_requested:
                print(f"  [auto_go_to_target] 收到退出信号，中止等待")
                return False

            # ★ 坐标取反: 蓝方前两个点需要反转后做距离判断
            if invert_coordinates:
                cx = -current_x
                cy = -current_y
                tx = -g_pos_target_x
                ty = -g_pos_target_y
            else:
                cx = current_x
                cy = current_y
                tx = g_pos_target_x
                ty = g_pos_target_y

            dx = tx - cx
            dy = ty - cy
            dist = math.sqrt(dx*dx + dy*dy)

            # yaw 误差检查
            yaw_ok = True
            if g_pos_target_yaw is not None:
                ew = abs(normalize_angle(g_pos_target_yaw - current_yaw))
                yaw_ok = ew < YAW_TOL
            else:
                ew = 0.0

            pos_ok = dist < POS_TOL
            all_ok = pos_ok and yaw_ok

            if all_ok:
                dwell_count += 1
                if dwell_count == 1:
                    yaw_info = f", yaw_err={ew:.1f} deg" if g_pos_target_yaw is not None else ""
                    print(f"  ⏳ 进入容差范围 (dist={dist:.3f} < {POS_TOL}{yaw_info}), 驻留确认中...")
                if dwell_count >= DWELL_CYCLES:
                    elapsed = time.time() - t0
                    yaw_info = f", yaw={current_yaw:.1f} deg" if g_pos_target_yaw is not None else ""
                    print(f"  ✓ 到达目标! 耗时 {elapsed:.1f}s, "
                          f"最终: ({cx:.3f}, {cy:.3f}{yaw_info}), "
                          f"误差: dx={dx:+.3f}, dy={dy:+.3f}")
                    # 零速驻留确保停稳
                    time.sleep(STOP_DWELL * DT)
                    return True
            else:
                dwell_count = 0

            # 每秒打印一次进度
            if int((time.time() - t0) * 2) % 2 == 0 and not hasattr(auto_go_to_target, '_last_print'):
                auto_go_to_target._last_print = 0
            now_sec = int(time.time() - t0)
            if now_sec > getattr(auto_go_to_target, '_last_print', 0):
                auto_go_to_target._last_print = now_sec
                yaw_info = f", yaw_err={ew:.1f} deg" if g_pos_target_yaw is not None else ""
                print(f"  [行进中] dist={dist:.3f}{yaw_info} | "
                      f"当前=({cx:.3f},{cy:.3f}) | "
                      f"已耗时 {now_sec}s")

            time.sleep(DT)

        print(f"  ✗ 到达超时 ({timeout}s), 最终误差: dist={dist:.3f}")
        return False


def auto_go_to_y(target_y, target_yaw=None, wait=True, timeout=120.0):
    """
    不动 X，只改变 Y 到目标值。
    X 固定在当前坐标 (current_x)，仅驱动 Y 轴到 target_y。

    参数:
        target_y: 目标 Y 坐标 (lidar 世界坐标系)
        target_yaw: 目标航向角 (deg), None=不控制yaw
        wait: True=阻塞等待到达, False=只设置目标立刻返回
        timeout: 到达超时时间(秒)

    返回: True=到达目标, False=超时/失败
    """
    return auto_go_to_target(target_x=current_x, target_y=target_y,
                             target_yaw=target_yaw, wait=wait, timeout=timeout)


def wait_weapon_finish(timeout=360.0):
    """
    等待电控发来 getweapon_finishflag == 1。
    等待期间持续发送 current_x, current_y, current_yaw（后台发送线程自动完成），
    不修改位置目标，不影响并发导航。

    参数:
        timeout: 超时时间(秒)

    返回: True=收到结束标志, False=超时
    """
    global getweapon_finishflag

    # ★ 清零旧标志，确保只响应本次武器拾取完成的新帧
    getweapon_finishflag = 0

    print(f"\n[wait_weapon_finish] 等待电控结束标志 (超时={timeout}s)...")
    print(f"  当前位置: x={current_x:.3f}, y={current_y:.3f}, yaw={current_yaw:.1f} deg")

    t0 = time.time()
    while time.time() - t0 < timeout:
        check_retry()
        if shutdown_requested:
            print("  [wait_weapon_finish] 收到退出信号，中止等待")
            return False

        if getweapon_finishflag == 1:
            elapsed = time.time() - t0
            print(f"  ✓ 收到结束标志! 耗时 {elapsed:.1f}s")
            return True

        # 每秒打印一次进度
        elapsed = time.time() - t0
        if int(elapsed) > getattr(wait_weapon_finish, '_last_print', -1):
            wait_weapon_finish._last_print = int(elapsed)
            print(f"  [等待中] 已等待 {int(elapsed)}s | "
                  f"flag={getweapon_finishflag} | "
                  f"x={current_x:.3f}, y={current_y:.3f}, yaw={current_yaw:.1f}")

        time.sleep(0.05)

    print(f"  ✗ 等待超时 ({timeout}s), flag={getweapon_finishflag}")
    return False


def continuous_send_loop():
    """
    后台线程：以 SEND_FREQ (100Hz) 持续发送 54字节 Computer_Frame_S 帧。

    关键逻辑:
      - Position_MeasureX/Y 始终 = current_x, current_y (实时里程计)
      - 当 position_mode_flag == 0 时:
          Position_Target_X/Y 自动跟随 current_x/y
          → 下位机看到 target == measure，位置环不动作
      - 当 position_mode_flag == 1 时:
          Position_Target_X/Y = auto_go_to_target 设定的值
          → 下位机做位置 PID 驱动底盘
      - ★ 串口断开自动重连（支持 ACM0↔ACM1 互换）
    """
    global ser, ser_lock
    global g_target_vx, g_target_vy, g_target_yaw
    global g_pos_target_x, g_pos_target_y, position_mode_flag
    print(f"持续发送线程启动 (频率={SEND_FREQ}Hz, 协议={SEND_FRAME_LEN}字节/帧)")

    reconnect_logged = False
    while not shutdown_requested:
        # ── 串口健康检查 & 自动重连 ──
        if ser is None or not ser.is_open:
            if not reconnect_logged:
                print("[发送线程] 串口不可用，等待重连...")
                reconnect_logged = True
            if not ensure_serial_connected():
                time.sleep(1.0)  # 等1秒再试，避免疯狂轮询
                continue
            print("[发送线程] 串口重连成功，恢复发送")
            reconnect_logged = False

        try:
            # ★ 位置模式开关: flag=0 时 target 自动跟随 current，下位机位置环不动作
            if position_mode_flag == 0:
                g_pos_target_x = current_x
                g_pos_target_y = current_y

            vx = g_target_vx
            vy = g_target_vy
            yaw = g_target_yaw if g_target_yaw is not None else current_yaw

            send_full_frame(vx, vy, yaw)
            reconnect_logged = False  # 发送成功，清除重连标记
            time.sleep(DT)
        except (serial.SerialException, OSError) as e:
            print(f"[发送线程] 发送异常 (串口可能断开): {e}")
            try:
                ser.close()
            except:
                pass
            ser = None
            time.sleep(0.5)
    print("持续发送线程退出")


def serial_read_loop():
    """后台线程：持续从串口读取下位机上报帧，更新 current_yaw（度）。
       串口断开时自动重连（支持 ACM0↔ACM1 互换）。"""
    global ser, ser_lock
    global current_yaw
    global retry_triggered, cancel_nav, retry_disabled, last_retry_flag
    buf = b''
    reconnect_logged = False
    while not shutdown_requested:
        # ── 串口健康检查 & 自动重连 ──
        if ser is None or not ser.is_open:
            if not reconnect_logged:
                print("[接收线程] 串口不可用，等待重连...")
                reconnect_logged = True
            if not ensure_serial_connected():
                time.sleep(1.0)
                continue
            print("[接收线程] 串口重连成功，恢复接收")
            reconnect_logged = False
            buf = b''  # 重连后清空缓冲区，避免解析到半帧

        try:
            if ser.in_waiting > 0:
                with ser_lock:
                    buf += ser.read(ser.in_waiting)
                while len(buf) >= RECV_FRAME_LEN:
                    idx = buf.find(bytes([RECV_HEADER]))
                    if idx < 0:
                        buf = b''
                        break
                    if idx > 0:
                        buf = buf[idx:]
                    if len(buf) < RECV_FRAME_LEN:
                        break
                    if buf[RECV_FRAME_LEN - 1] == RECV_TAIL:
                        yaw_deg = struct.unpack('<f', buf[5:9])[0]  # yaw在byte[5:9]
                        current_yaw = yaw_deg
                        global calib_done
                        calib_done = bool(buf[1] & CALIB_DONE_MASK)
                        global step_mode_feedback
                        step_mode_feedback = buf[2] if buf[2] in (0, 1, 2, 3) else 0
                        global getweapon_finishflag
                        getweapon_finishflag = buf[3]
                        global getkfs_flag
                        getkfs_flag = buf[4]       # GetKFS_Flag在byte[4]
                        global retry_flag
                        retry_flag = buf[9]         # ★ Retry_Flag在byte[9]
                        # ★ retry 边沿触发: retry_flag 值变化时才锁存
                        if retry_flag in (0x01, 0x02, 0x03, 0x04, 0x05) and retry_flag != last_retry_flag and not retry_triggered and not retry_disabled:
                            last_retry_flag = retry_flag
                            retry_triggered = retry_flag
                            cancel_nav = True
                            print(f"\n{'!'*60}")
                            print(f"[串口收] ★★★ 检测到 Retry_Flag=0x{retry_flag:02X}，触发任务切换! ★★★")
                            print(f"{'!'*60}\n")
                        elif retry_flag != last_retry_flag:
                            last_retry_flag = retry_flag  # 值变化就更新（包括变0），为下次同值触发做准备
                        if not hasattr(serial_read_loop, '_diag_cnt'):
                            serial_read_loop._diag_cnt = 0
                        if serial_read_loop._diag_cnt < 5:
                            serial_read_loop._diag_cnt += 1
                            print(f"[串口收] 帧#{serial_read_loop._diag_cnt}: getweapon_finishflag={getweapon_finishflag:<5.1f}yaw={yaw_deg:.2f} deg calib={calib_done} retry={retry_flag:#04x}")
                        buf = buf[RECV_FRAME_LEN:]
                    else:
                        buf = buf[1:]
                reconnect_logged = False  # 成功读到数据，清除重连标记
            else:
                time.sleep(0.001)
        except (serial.SerialException, OSError) as e:
            if shutdown_requested:
                break
            print(f"[接收线程] 串口异常 (可能断开): {e}")
            try:
                ser.close()
            except:
                pass
            ser = None
            buf = b''
            time.sleep(0.5)
        except Exception as e:
            if shutdown_requested:
                break
            print(f"串口接收异常: {e}")
            time.sleep(0.01)
            
            
            
            
            
def close_serial():
    global ser
    try:
        if ser is not None and ser.is_open:
            ser.close()
            print("串口已关闭")
    except Exception as e:
        print(f"串口关闭异常: {e}")
    ser = None



def set_step_mode(val: int, timeout=30.0):
    """设置台阶模式并等待下位机确认。
    使用 cmd_lift 发送指令:
      - val=1: 发 cmd_lift=1 → 等 feedback=1 → 发 cmd_lift=0 → 等 feedback=0 → 结束 (上台阶)
      - val=2: 发 cmd_lift=2 → 等 feedback=2 → 发 cmd_lift=0 → 等 feedback=0 → 结束 (下台阶)
      - val=3: 发 cmd_lift=3，立即返回，一直保持3直到再次调用 set_step_mode 改变 (下压模式)
      - val=0: 发 cmd_lift=0，立即返回
    期间 target_x/y 持续跟随 current_x/y，底盘不移动。
    """
    global cmd_lift, step_mode_feedback
    global g_pos_target_x, g_pos_target_y
    global g_target_yaw, g_pos_target_yaw

    if val not in [0, 1, 2, 3]:
        print(f"set_step_mode: 无效参数 {val}, 忽略")
        return False

    # ★ 锁住进入函数时的目标 yaw，台阶执行期间持续发送
    _hold_yaw = g_target_yaw

    if val == 0:
        cmd_lift = 0
        print(f"set_step_mode({val}): 停止台阶, cmd_lift=0")
        return True

    if val == 3:
        # 下压模式: 设置后立即返回，cmd_lift 保持为 3 不等待反馈、不复位
        cmd_lift = 3
        print(f"set_step_mode({val}): 下压模式, cmd_lift=3 (持续保持, 不等待反馈)")
        return True

    print(f"\n[set_step_mode] 阶段1: 发送 cmd_lift={val}, 等待 feedback={val}...")
    print(f"  当前位置: x={current_x:.3f}, y={current_y:.3f}")

    # ── 阶段1: 持续发 cmd_lift=val，等待下位机回报 step_mode_feedback==val ──
    cmd_lift = val
    t0 = time.time()
    while time.time() - t0 < timeout:
        check_retry()
        if shutdown_requested:
            cmd_lift = 0
            return False

        # 保持 target 跟随 current，底盘不移动；yaw 保持进入时的值
        g_pos_target_x = current_x
        g_pos_target_y = current_y
        g_pos_target_yaw = _hold_yaw
        g_target_yaw = _hold_yaw

        if step_mode_feedback == val:
            elapsed = time.time() - t0
            print(f"  ✓ 阶段1完成: feedback={val} (耗时 {elapsed:.1f}s)")
            break
        time.sleep(0.01)
    else:
        cmd_lift = 0
        print(f"  ✗ 阶段1超时 ({timeout}s), feedback={step_mode_feedback} (期望={val})")
        return False

    # ── 阶段2: 发 cmd_lift=0，等待下位机回报 step_mode_feedback==0 ──
    cmd_lift = 0
    print(f"  阶段2: 发送 cmd_lift=0, 等待 feedback=0...")
    t0 = time.time()
    while time.time() - t0 < timeout:
        check_retry()
        if shutdown_requested:
            return False

        # 保持 target 跟随 current，底盘不移动；yaw 保持进入时的值
        g_pos_target_x = current_x
        g_pos_target_y = current_y
        g_pos_target_yaw = _hold_yaw
        g_target_yaw = _hold_yaw

        if step_mode_feedback == 0:
            elapsed = time.time() - t0
            print(f"  ✓ 阶段2完成: feedback=0 (耗时 {elapsed:.1f}s)")
            print(f"  ✓ set_step_mode({val}) 全部完成! 总耗时 {time.time() - t0 + elapsed:.1f}s")
            return True
        time.sleep(0.01)

    print(f"  ✗ 阶段2超时 ({timeout}s), feedback={step_mode_feedback} (期望=0)")
    return False


def run_getkfs(cmd: int, timeout=30.0):
    """执行KFS拾取/放置操作 (吸盘控制握手协议)。

    参数:
        cmd: 操作指令
            1 → 向前拾取KFS        (GetKFS_CMD=0x01, 等待 GetKFS_Flag=0x01)
            2 → 向下拾取KFS        (GetKFS_CMD=0x02, 等待 GetKFS_Flag=0x02)
            3 → 向外侧放置KFS      (GetKFS_CMD=0x03, 等待 GetKFS_Flag=0x03)
            4 → 向前吸取并存放KFS  (GetKFS_CMD=0x04, 等待 GetKFS_Flag=0x04)
            5 → 向下吸取并存放KFS  (GetKFS_CMD=0x05, 等待 GetKFS_Flag=0x05)
            6 → 吸取高位KFS        (GetKFS_CMD=0x06, 等待 GetKFS_Flag=0x06)
            7 → 操作7              (GetKFS_CMD=0x07, 等待 GetKFS_Flag=0x07, 再发0x10停止)
            8 → 操作8              (GetKFS_CMD=0x08, 等待 GetKFS_Flag=0x08, 再发0x10停止)
        timeout: 每阶段超时时间(秒)

    流程 (cmd=1~6):
        阶段1: 设置 GetKFS_CMD = cmd → 等待下位机回报 GetKFS_Flag == cmd
        阶段2: 设置 GetKFS_CMD = 0   → 等待下位机回报 GetKFS_Flag == 0 → 函数返回

    流程 (cmd=7,8):
        阶段1: 设置 GetKFS_CMD = cmd → 发送1秒 (不等待回报)
        阶段2: 设置 GetKFS_CMD = 0x10 → 等待下位机回报 GetKFS_Flag == 0
        阶段3: 设置 GetKFS_CMD = 0x00 → 等待下位机回报 GetKFS_Flag == 0 → 函数返回

    返回: True=握手完成, False=超时/失败
    """
    global GetKFS_CMD

    if cmd not in [1, 2, 3, 4, 5, 6,7,8,9]:
        print(f"run_getkfs: 无效参数 {cmd}, 仅支持 1/2/3/4/5/6/7/8, 忽略")
        return False

    cmd_names = {1: "向前拾取", 2: "向下拾取", 3: "向外侧放置",
                 4: "向前吸取并存放KFS", 5: "向下吸取并存放KFS", 6: "吸取高位KFS",
                 7: "操作7",8: "操作8",9: "操作9"}
    print(f"\n[run_getkfs] 阶段1: 发送 GetKFS_CMD=0x{cmd:02X} ({cmd_names[cmd]})"
          + (f", 等待 GetKFS_Flag=0x{cmd:02X}..." if cmd not in (7, 8) else ", 持续发1秒后立即发0x10..."))
    print(f"  当前位置: x={current_x:.3f}, y={current_y:.3f}, yaw={current_yaw:.1f} deg")

    # ── 阶段1 ──
    GetKFS_CMD = cmd
    if cmd in (7, 8):
        # 0x07/0x08: 发1秒，不等待下位机回报
        t0 = time.time()
        while time.time() - t0 < 1.0:
            check_retry()
            if shutdown_requested:
                GetKFS_CMD = 0x00
                return False
            time.sleep(0.01)
        print(f"  ✓ 阶段1完成: 已发送 GetKFS_CMD=0x{cmd:02X} (持续1秒)")
    else:
        # 1~6: 等待下位机回报 getkfs_flag==cmd
        t0 = time.time()
        while time.time() - t0 < timeout:
            check_retry()
            if shutdown_requested:
                GetKFS_CMD = 0x00
                return False

            if getkfs_flag == cmd:
                elapsed = time.time() - t0
                print(f"  ✓ 阶段1完成: GetKFS_Flag=0x{getkfs_flag:02X} (耗时 {elapsed:.1f}s)")
                break
            time.sleep(0.01)
        else:
            GetKFS_CMD = 0x00
            print(f"  ✗ 阶段1超时 ({timeout}s), GetKFS_Flag=0x{getkfs_flag:02X} (期望=0x{cmd:02X})")
            return False

    # ── 阶段2 (仅 0x07/0x08): 发 0x10 停止，等待 getkfs_flag==0 ──
    if cmd in (7, 8):
        GetKFS_CMD = 0x10
        print(f"  阶段2: 发送 GetKFS_CMD=0x10, 等待 GetKFS_Flag=0x00...")
        t0 = time.time()
        while time.time() - t0 < timeout:
            check_retry()
            if shutdown_requested:
                GetKFS_CMD = 0x00
                return False

            if getkfs_flag == 0x00:
                elapsed = time.time() - t0
                print(f"  ✓ 阶段2完成: GetKFS_Flag=0x00 (耗时 {elapsed:.1f}s)")
                break
            time.sleep(0.01)
        else:
            GetKFS_CMD = 0x00
            print(f"  ✗ 阶段2超时 ({timeout}s), GetKFS_Flag=0x{getkfs_flag:02X} (期望=0x00)")
            return False

    # ── 阶段3: 发 GetKFS_CMD=0x00，等待下位机回报 getkfs_flag==0x00 ──
    GetKFS_CMD = 0x00
    stage_label = "阶段3" if cmd in (7, 8) else "阶段2"
    print(f"  {stage_label}: 发送 GetKFS_CMD=0x00, 等待 GetKFS_Flag=0x00...")
    t0 = time.time()
    while time.time() - t0 < timeout:
        check_retry()
        if shutdown_requested:
            return False

        if getkfs_flag == 0x00:
            elapsed = time.time() - t0
            print(f"  ✓ {stage_label}完成: GetKFS_Flag=0x00 (耗时 {elapsed:.1f}s)")
            print(f"  ✓ run_getkfs({cmd}) 全部完成!")
            return True
        time.sleep(0.01)

    print(f"  ✗ {stage_label}超时 ({timeout}s), GetKFS_Flag=0x{getkfs_flag:02X} (期望=0x00)")
    return False



# ===================== 角度工具函数 =====================
def quat2yaw(qx, qy, qz, qw):
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle_deg):
    """角度归一化到 [-180, 180] 度"""
    angle_deg = angle_deg % 360.0
    if angle_deg > 180.0:
        angle_deg -= 360.0
    return angle_deg


# ===================== ROS 里程计订阅节点 =====================
class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_sub_node')
        self.subscription = self.create_subscription(
            Odometry,
            '/fastlio2/lio_odom',
            self.odom_callback,
            100
        )
        self.callback_count = 0
        # ===== /to_meiling publisher =====
        self.meiling_pub = self.create_publisher(Target, "/to_meiling", 10)
        # ===== subscribe to Meiling status topic =====
        self.meiling_sub = self.create_subscription(
            Fas,
            "/Das",
            self.meiling_callback,
            10
        )
        # ===== 定时发布当前位置到 /to_meiling (50Hz) =====
        self.meiling_timer = self.create_timer(0.02, self.timer_publish_to_meiling)

    def odom_callback(self, msg):
        global current_x, current_y
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        self.callback_count += 1
        if self.callback_count % 50 == 0:
            self.get_logger().info(f"里程计更新: x={current_x:.3f}, y={current_y:.3f}, yaw(串口)={current_yaw:.1f}°")

    def timer_publish_to_meiling(self):
        self.publish_to_meiling(current_x, current_y, current_yaw)

    def publish_to_meiling(self, x, y, yaw):
        msg = Target()
        msg.x = x
        msg.y = y
        msg.yaw = yaw
        self.meiling_pub.publish(msg)
        self.get_logger().info(f"to Meiling: x={x} y={y} yaw={yaw}")

    def meiling_callback(self, msg):
        self.get_logger().info(f"from /fas: status={msg.status} zq={msg.zq} yaw={msg.yaw}")

def start_livox_driver(wait=1.0):
    """启动 livox 雷达驱动。wait: 启动后等待秒数（默认1s，原5s太保守）"""
    global livox_process
    print("\n启动 livox_ros_driver2 (MID360) ...")
    livox_process = subprocess.Popen(
        ["ros2", "launch", "livox_ros_driver2", "msg_MID360_launch.py"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    time.sleep(wait)
    print(f"livox_ros_driver2 启动完成 (等待 {wait}s)")


def stop_livox_driver():
    global livox_process
    if livox_process is not None:
        try:
            livox_process.terminate()
            livox_process.wait(timeout=3)
            subprocess.run(["pkill", "-f", "livox_ros_driver2"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print("livox_ros_driver2 进程已关闭")
        except:
            try:
                livox_process.kill()
            except:
                pass
    livox_process = None


# ===================== 启动/关闭 FAST-LIO2 =====================
def start_fastlio2(wait=1.0):
    """启动 fastlio2 里程计。wait: 启动后等待秒数（默认1s，原5s太保守）"""
    global lio_process
    print("\n启动 fastlio2 ...")
    lio_process = subprocess.Popen(
        ["ros2", "launch", "fastlio2", "lio_launch.py"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    time.sleep(wait)
    print(f"fastlio2 启动完成 (等待 {wait}s)")


def stop_fastlio2():
    global lio_process
    if lio_process is not None:
        try:
            lio_process.terminate()
            lio_process.wait(timeout=3)
            subprocess.run(["pkill", "-f", "fastlio2"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print("fastlio2 进程已关闭")
        except:
            try:
                lio_process.kill()
            except:
                pass
    lio_process = None


# ===================== ★ 统一雷达系统 停止/启动/等待 =====================
def stop_radar_system():
    """统一停止雷达+里程计（先停车，再停 fastlio2，最后停 livox）"""
    print("\n" + "=" * 60)
    print("[雷达系统] 停止雷达 + 里程计...")
    print("=" * 60)
    stop_car()
    stop_fastlio2()
    time.sleep(0.3)           # 等 fastlio2 进程退出
    stop_livox_driver()
    time.sleep(0.5)           # 等 livox 设备释放
    print("[雷达系统] 停止完成\n")


def start_radar_system(livox_wait=1.0, lio_wait=1.0):
    """统一启动雷达+里程计，和主程序完全一样的调用方式"""
    start_livox_driver(wait=livox_wait)
    start_fastlio2(wait=lio_wait)


def wait_odom_ready(timeout=5.0):
    """等待新里程计数据就绪，就绪后同步位置目标到当前坐标。
       返回: True=就绪, False=超时"""
    global last_odom_restart_time, odom_last_time
    global g_pos_target_x, g_pos_target_y
    last_odom_restart_time = time.time()
    odom_last_time = 0.0

    print("[雷达系统] 等待新里程计数据...")
    waited = 0.0
    while waited < timeout and not shutdown_requested:
        if odom_last_time > 0:
            g_pos_target_x = current_x
            g_pos_target_y = current_y
            print(f"[雷达系统] 新里程计就绪, 位置=({current_x:.3f}, {current_y:.3f}), 目标已同步")
            return True
        time.sleep(0.01)
        waited += 0.01

    # 超时也强制同步
    g_pos_target_x = current_x
    g_pos_target_y = current_y
    print(f"[雷达系统] 等待超时 ({timeout}s)，强制同步目标=({current_x:.3f}, {current_y:.3f})")
    return False


def restart_radar_system():
    """完整重启雷达系统: 停止 → 启动 → 等待里程计就绪。
       和主程序启动方式完全一致: start_livox_driver() + start_fastlio2()"""
    stop_radar_system()
    start_radar_system()
    wait_odom_ready()
    print("[雷达系统] 重启完成\n")


def restart_lidar_odom():
    """重启 livox 雷达驱动 + fastlio2 里程计"""
    global last_odom_restart_time, odom_interval_exceeded, odom_last_time, odom_max_interval
    global g_pos_target_x, g_pos_target_y
    now = time.time()
    if now - last_odom_restart_time < ODOM_RESTART_COOLDOWN:
        print(f"[健康监控] 冷却中，跳过重启 (距上次 {now - last_odom_restart_time:.1f}s < {ODOM_RESTART_COOLDOWN}s)")
        odom_interval_exceeded = False
        return

    # ★ 重启前先停车，防止下位机在里程计离线期间乱动
    print("[健康监控] 停车并准备重启...")
    stop_car()

    print("\n" + "=" * 60)
    print("[健康监控] 检测到里程计间隔 > {:.0f}ms，重启雷达驱动 + 里程计...".format(ODOM_MAX_INTERVAL * 1000))
    print("=" * 60)
    stop_fastlio2()
    time.sleep(0.2)
    stop_livox_driver()
    time.sleep(0.2)
    start_livox_driver()
    start_fastlio2()
    last_odom_restart_time = time.time()
    odom_interval_exceeded = False
    odom_max_interval = 0.0
    odom_last_time = 0.0  # 重置，避免启动期间的间隔触发误报

    # ★ 重启后等待新里程计入来，立即同步位置目标到新坐标，防止位置跳变导致下位机暴冲
    print("[健康监控] 等待新里程计数据...")
    waited = 0.0
    while waited < 10.0 and not shutdown_requested:
        if odom_last_time > 0:
            # 新里程计数据已到达（odom_last_time 在 callback 中被更新，重启时被重置为 0）
            g_pos_target_x = current_x
            g_pos_target_y = current_y
            print(f"[健康监控] 新里程计就绪, 位置=({current_x:.3f}, {current_y:.3f}), 目标已同步")
            break
        time.sleep(0.01)
        waited += 0.01
    else:
        # 超时也强制同步
        g_pos_target_x = current_x
        g_pos_target_y = current_y
        print(f"[健康监控] 等待超时，强制同步目标=({current_x:.3f}, {current_y:.3f})")

    print("[健康监控] 重启完成\n")


def wait_odom_healthy(stable_time=5.0):
    """
    启动前等待：持续监控里程计间隔，max > 阈值时由健康监控重启，
    直到连续 stable_time 秒内 max < 阈值才返回。
    """
    global odom_interval_exceeded, odom_max_interval
    print(f"\n[启动检查] 等待里程计稳定 (需连续 {stable_time}s 内 max < {ODOM_MAX_INTERVAL*1000:.0f}ms)...")
    odom_interval_exceeded = False
    odom_max_interval = 0.0
    stable_start = time.time()
    last_print = 0
    while not shutdown_requested:
        if odom_interval_exceeded:
            print(f"  [启动检查] 间隔超限 (max={odom_max_interval*1000:.0f}ms > {ODOM_MAX_INTERVAL*1000:.0f}ms), "
                  f"等待健康监控重启...")
            odom_interval_exceeded = False
            odom_max_interval = 0.0
            stable_start = time.time()
            last_print = 0
        else:
            elapsed = time.time() - stable_start
            if elapsed >= stable_time:
                print(f"  [启动检查] ✓ 里程计稳定 (max={odom_max_interval*1000:.0f}ms < {ODOM_MAX_INTERVAL*1000:.0f}ms, "
                      f"已持续 {elapsed:.1f}s)")
                return True
            if int(elapsed) > last_print:
                last_print = int(elapsed)
                print(f"  [启动检查] 稳定中... {elapsed:.0f}/{stable_time:.0f}s, "
                      f"当前 max={odom_max_interval*1000:.0f}ms")
        time.sleep(0.5)
    return False


def odom_health_monitor():
    """后台线程：每 0.5s 检查里程计间隔是否超阈值，超则重启雷达+里程计"""
    print(f"[健康监控] 启动 (阈值 > {ODOM_MAX_INTERVAL*1000:.0f}ms, 冷却 {ODOM_RESTART_COOLDOWN}s)")
    while not shutdown_requested:
        if odom_interval_exceeded:
            restart_lidar_odom()
        time.sleep(0.5)


# ===================== ★ 原地旋转阶段执行器 =====================
def run_rotate_stage(target_yaw):
    """
    原地旋转到目标 yaw 角。
    target_x / target_y 锁定为当前坐标（底盘不移动），仅旋转 yaw。
    yaw 误差 ≤ 1 度时停止。

    参数:
        target_yaw: 目标航向角 (deg)

    返回: True=到达目标, False=超时/失败
    """
    global cancel_nav, g_pos_target_x, g_pos_target_y, g_pos_target_yaw, g_target_yaw

    yaw_tol = 1.0          # yaw 误差阈值 (度)

    # 锁住当前坐标，只改 yaw
    g_pos_target_x = current_x
    g_pos_target_y = current_y
    g_pos_target_yaw = float(target_yaw)
    g_target_yaw = g_pos_target_yaw

    print(f"\n[原地旋转] 目标 yaw={target_yaw:.1f} deg, 当前 yaw={current_yaw:.1f} deg")
    print(f"  锁住位置: ({current_x:.3f}, {current_y:.3f})")

    dwell_count = 0
    t0 = time.time()
    timeout = 60.0

    while time.time() - t0 < timeout:
        check_retry()
        if cancel_nav:
            print("  [原地旋转] ★ 收到取消信号，中止旋转")
            return False
        if shutdown_requested:
            print("  [原地旋转] 收到退出信号")
            return False

        # ★ 每周期更新 target 为当前坐标，跟随 LIO 漂移，确保底盘原地不动
        g_pos_target_x = current_x
        g_pos_target_y = current_y

        ew = abs(normalize_angle(target_yaw - current_yaw))

        if ew <= yaw_tol:
            dwell_count += 1
            if dwell_count == 1:
                print(f"  ⏳ yaw 进入容差 (err={ew:.1f} deg ≤ {yaw_tol}), 驻留确认中...")
            if dwell_count >= DWELL_CYCLES:
                elapsed = time.time() - t0
                print(f"  ✓ 旋转完成! 耗时 {elapsed:.1f}s, "
                      f"最终 yaw={current_yaw:.1f} deg, 误差={ew:.1f} deg")
                time.sleep(STOP_DWELL * DT)
                return True
        else:
            dwell_count = 0

        # 每秒打印一次进度
        elapsed = time.time() - t0
        if int(elapsed) > getattr(run_rotate_stage, '_last_print', -1):
            run_rotate_stage._last_print = int(elapsed)
            print(f"  [旋转中] yaw_err={ew:.1f} deg | "
                  f"当前 yaw={current_yaw:.1f} deg | 已耗时 {int(elapsed)}s")

        time.sleep(DT)

    ew = abs(normalize_angle(target_yaw - current_yaw))
    print(f"  ✗ 旋转超时 ({timeout}s), 最终误差={ew:.1f} deg")
    return False


# ===================== 全局资源释放 =====================
def clean_resources(node, spin_thread, send_thread=None, read_thread=None):
    global shutdown_requested
    shutdown_requested = True

    stop_car()
    time.sleep(0.1)

    # ★ 先等待后台线程自然退出（它们看到 shutdown_requested 就会退出循环）
    for t_name, t in [("send", send_thread), ("read", read_thread)]:
        if t is not None and t.is_alive():
            t.join(timeout=2)
            if t.is_alive():
                print(f"⚠ {t_name} 线程未能在2秒内退出")

    # ★ 线程都停了再关串口，避免 Bad file descriptor
    close_serial()

    stop_fastlio2()
    stop_livox_driver()


    if node is not None:
        try:
            node.destroy_node()
        except:
            pass
    try:
        rclpy.shutdown()
    except:
        pass
    if spin_thread is not None and spin_thread.is_alive():
        spin_thread.join(timeout=2)
    print("\n所有资源已清理完毕")


# ===================== ★ retry 恢复任务 =====================
def run_retry_mission(retry_val: int, team: str, config: dict):
    """根据 retry_flag 值和队伍颜色执行对应的恢复任务，完成后不返回（直接退出程序）"""
    global retry_triggered, cancel_nav, retry_disabled
    global g_pos_target_x, g_pos_target_y, odom_last_time, last_odom_restart_time
    retry_triggered = 0   # 清除触发标志，让新任务正常执行
    cancel_nav = False    # 清除取消标志，否则 run_rotate_stage 会立即返回
    retry_disabled = False  # ★ 重新允许 retry，否则第二次 retry 会被忽略

    name = {0x01: "武馆重试→梅林", 0x02: "单项赛上3区", 0x03: "对抗赛3区", 0x04: "retry 0x04", 0x05: "retry 0x05"}
    print(f"\n{'='*60}")
    print(f"  执行 Retry 任务: 0x{retry_val:02X} ({name.get(retry_val, '未知')}) — {team}")
    print(f"{'='*60}")

    if team == "红方":
        # ==================== 红方 retry 任务 ====================
        if retry_val == 0x01:
            # ── 0x01: 武馆重试到梅林 ──
            auto_go_to_target(target_x=2.042, target_y=-1.446, target_yaw=0.0 )#点3
            #2.561,-0.239
            #2.616,-1.446
            #2.561,-2.653
            #加入看2号口KFS
            # === 路径规划：根据地形配置自动生成指令 ===
            print("[红方 retry 0x01] ★ auto_go_to_target 已返回, 即将调用 plan_route...")
            plan = plan_route("红方", config)
            if plan is None:
                print("[红方] 路径规划失败！")
                return
            execute_commands(plan["pre_cmds"])
            execute_commands(plan["main_cmds"])
            # ==========================================
            run_rotate_stage(0)
            auto_go_to_target(target_x=8.363, target_y=-2.754, target_yaw=0.0 )
            auto_go_to_target(target_x=8.363, target_y=-4.468, target_yaw=0.0 )
            set_step_mode(3)
            sleep_check_retry(2)
            auto_go_to_target(target_x=11.356, target_y=-4.468, target_yaw=0.0 )
    
            sleep_check_retry(100)
    
    
    
        elif retry_val == 0x02:
            # ── 红方 0x02: 整个流程重试──
            run_red_mission(config)



        elif retry_val == 0x03:
            # ── 红方 0x03: 对抗赛3区 左──
            restart_radar_system()
            auto_go_to_target(target_x=4.577,target_y=0.102, target_yaw=0.0 )
            auto_go_to_target(target_x=4.577,target_y=1.250, target_yaw=0.0 )
            time.sleep(2)
            run_getkfs(3)
            sleep_check_retry(100)
        elif retry_val == 0x04:
            # ── 红方 0x04: 对抗赛3区中 ──
            restart_radar_system()
            auto_go_to_target(target_x=4.577,target_y=0.102, target_yaw=0.0 )
            auto_go_to_target(target_x=4.577, target_y=0.738, target_yaw=0.0)
            time.sleep(2)
            run_getkfs(3)
            print("  [run_retry_mission] retry 0x04 — 待实现")
            sleep_check_retry(100)

        elif retry_val == 0x05:
            # ── 红方 0x05: 对抗赛3区右──
            restart_radar_system()
            auto_go_to_target(target_x=4.577,target_y=0.102, target_yaw=0.0 )
            auto_go_to_target(target_x=4.577,target_y=0.150, target_yaw=0.0 )
            time.sleep(2)
            run_getkfs(3)

            print("  [run_retry_mission] retry 0x05 — 待实现")
            sleep_check_retry(100)
        else:
            print(f"  [run_retry_mission] 未知 retry_val=0x{retry_val:02X}, 无对应任务")

    else:  # 蓝方
        # ==================== 蓝方 retry 任务 ====================
        if retry_val == 0x01:
            # ── 0x01: 武馆重试到梅林 ──
            auto_go_to_target(target_x=2.042, target_y=1.551, target_yaw=0.0 )#改为梅林中心点，x轴要改！！！！！！！！！！!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            # === 路径规划：根据地形配置自动生成指令 ===
            print("[蓝方 retry 0x01] ★ auto_go_to_target 已返回, 即将调用 plan_route...")
            plan = plan_route("蓝方", config)
            if plan is None:
                print("[蓝方] 路径规划失败！")
                return
            execute_commands(plan["pre_cmds"])
            execute_commands(plan["main_cmds"])
            run_rotate_stage(0)
            auto_go_to_target(target_x=8.253,target_y=2.157, target_yaw=0.0 )
            set_step_mode(3)
            sleep_check_retry(2)
            auto_go_to_target(target_x=10.958,target_y=3.7, target_yaw=0.0 )
    #下梅林点位(8.253,2.157)！！！！！！！！！！！!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    #上斜坡点位(10.958,3.7)!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            sleep_check_retry(100)
        elif retry_val == 0x02:
            # ── 蓝方 0x02: 对抗赛重试 ──
            run_blue_mission(config)
        elif retry_val == 0x03:
            # ── 蓝方 0x03: 对抗赛3区 左──
            restart_radar_system()
            auto_go_to_target(target_x=4.577, target_y=-0.346, target_yaw=0.0)
            auto_go_to_target(target_x=4.577, target_y=-0.538, target_yaw=0.0)
            time.sleep(2)
            run_getkfs(3)
            sleep_check_retry(100)
        elif retry_val == 0x04:
            # ── 蓝方 0x04: 对抗赛3区中点 ──
            restart_radar_system()
            auto_go_to_target(target_x=4.577, target_y=-0.346, target_yaw=0.0)
            auto_go_to_target(target_x=4.577, target_y=-1.016, target_yaw=0.0)
            time.sleep(2)
            run_getkfs(3)
            print("  [run_retry_mission] retry 0x04 — 待实现")
            sleep_check_retry(100)

        elif retry_val == 0x05:
            # ── 蓝方 0x05: 对抗赛3区右点 ──
            restart_radar_system()
            auto_go_to_target(target_x=4.577, target_y=-0.346, target_yaw=0.0)
            auto_go_to_target(target_x=4.577, target_y=-1.599, target_yaw=0.0)
            run_getkfs(3)
            sleep_check_retry(1)
            print("  [run_retry_mission] retry 0x05 — 待实现")
            sleep_check_retry(100)
        else:
            print(f"  [run_retry_mission] 未知 retry_val=0x{retry_val:02X}, 无对应任务")

    print(f"\n{'='*60}")
    print(f"  Retry 任务 0x{retry_val:02X} ({team}) 完成，程序退出")
    print(f"{'='*60}")


# ===================== 命令执行器 =====================
def execute_commands(cmds):
    """执行规划器生成的结构化指令序列。

    参数:
        cmds: [(func_name, args_tuple, kwargs_dict), ...]
              例: [("run_rotate_stage", (90,), {}),
                   ("auto_go_to_target", (), {"target_x": 2.5, "target_y": 0.0, "target_yaw": 0.0})]

    返回: True=全部执行完成, False=某条指令失败
    """
    dispatch = {
        "auto_go_to_target": lambda a, kw: auto_go_to_target(*a, **kw),
        "run_getkfs":         lambda a, kw: run_getkfs(*a, **kw),
        "set_step_mode":      lambda a, kw: set_step_mode(*a, **kw),
        "run_rotate_stage":   lambda a, kw: run_rotate_stage(*a, **kw),
        "sleep_check_retry":  lambda a, kw: sleep_check_retry(*a, **kw),
    }

    for i, (func_name, args, kwargs) in enumerate(cmds):
        # 格式化指令用于日志
        arg_strs = [repr(a) for a in args]
        arg_strs += [f"{k}={v!r}" for k, v in kwargs.items()]
        print(f"\n  [{i+1}/{len(cmds)}] {func_name}({', '.join(arg_strs)})")

        if func_name not in dispatch:
            print(f"  ✗ 未知指令: {func_name}，跳过")
            continue

        check_retry()  # 每条指令执行前检查 retry
        result = dispatch[func_name](args, kwargs)
        # 注意: 部分函数返回 None (无明确返回值), 此处不做强制判断
        # 若函数内部超时/失败会自行打印错误信息

    print(f"\n  ✓ 指令序列执行完成 (共 {len(cmds)} 条)")
    return True


# ===================== 红蓝双方任务序列 =====================
def run_red_mission(config=None):
    """红方任务序列。config 为地形配置 {(r,c): 't'/'f'/'n'}，None 则用默认。"""
    global getweapon_startflag
    global AimtoGetKFSFlag
    global GetKFS_CMD
    global getweapon_startflag
    global invert_coordinates
    print("\n========== 执行红方任务序列 ==========")
    getweapon_startflag=1#发开始标志位
    auto_go_to_target(target_x=0.241, target_y=0.906,  target_yaw=0.0)
    time.sleep(1)
    print("已到达拿武器目标点")
    getweapon_startflag=0 #发结束标志位
    # ★ 导航到目标点（后台线程），同时等待武器完成标志
    #     武器完成时立即中断导航；武器超时则继续等待导航到达
    nav_thread = threading.Thread(
        target=lambda: auto_go_to_target(target_x=0.932, target_y=-0.077, target_yaw=0.0),
        daemon=True
    )
    nav_thread.start()
    sleep_check_retry(0.1)  # 让导航线程先设定目标

    print("已发送结束标志位") # ★ 清零旧标志，防止读到之前帧的残留值

    if wait_weapon_finish():
        # 武器完成 → 立即停止导航
        print("[红方] 武器完成! 立即停止导航")
        stop_auto_nav()
        nav_thread.join(timeout=3)
    else:
        # 武器等待超时 → 继续等待导航到达目标
        print("[红方] 武器等待超时，等待导航到达目标...")
        nav_thread.join()    
    auto_go_to_target(target_x=0.994, target_y=-1.606, target_yaw=0.0 )
    sleep_check_retry(5)
    auto_go_to_target(target_x=2.442, target_y=-1.606, target_yaw=0.0 )#点3
    #2.561,-0.239
    #2.616,-1.446
    #2.561,-2.653
    #加入看2号口KFS
    # === 路径规划：根据地形配置自动生成指令 ===
    plan = plan_route("红方", config)
    if plan is None:
        print("[红方] 路径规划失败！")
        return
    execute_commands(plan["pre_cmds"])
    execute_commands(plan["main_cmds"])
    # ==========================================
    run_rotate_stage(0)
    auto_go_to_target(target_x=8.363, target_y=-2.754, target_yaw=0.0 )
    auto_go_to_target(target_x=8.363, target_y=-4.428, target_yaw=0.0 )
    set_step_mode(3)
    sleep_check_retry(2)
    auto_go_to_target(target_x=10.8, target_y=-4.428, target_yaw=0.0 )
    set_step_mode(0)
    sleep_check_retry(100)
    print("========== 红方任务序列完成 ==========")


def run_blue_mission(config=None):
    """蓝方任务序列。config 为地形配置 {(r,c): 't'/'f'/'n'}，None 则用默认。"""
    if config is None:
        config = BLUE_CONFIG_DEFAULT
    global getweapon_startflag
    global AimtoGetKFSFlag
    global GetKFS_CMD
    global getweapon_startflag
    global invert_coordinates
    print("\n========== 执行蓝方任务序列 ==========")
    run_rotate_stage(180)
    # ── 前两个点: 坐标取反 ──
    invert_coordinates = True   # ★ 开启坐标取反: 串口帧和到达判断都用取反后的xy
    getweapon_startflag=1#发开始标志位
    auto_go_to_target(target_x=-0.484, target_y=-0.147,  target_yaw=180.0)#拿武器目标点,要改!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    time.sleep(1)
    print("已到达拿武器目标点")
    getweapon_startflag=0 #发结束标志位
    # ★ 导航到目标点（后台线程），同时等待武器完成标志
    #     武器完成时立即中断导航；武器超时则继续等待导航到达
    nav_thread = threading.Thread(
        target=lambda: auto_go_to_target(target_x=-0.269, target_y=1.056, target_yaw=0.0),
        daemon=True
    )
    nav_thread.start()
    sleep_check_retry(0.1)  # 让导航线程先设定目标

    print("已发送结束标志位") # ★ 清零旧标志，防止读到之前帧的残留值

    if wait_weapon_finish():
        # 武器完成 → 立即停止导航
        print("[蓝方] 武器完成! 立即停止导航")
        stop_auto_nav()
        nav_thread.join(timeout=3)
    else:
        # 武器等待超时 → 继续等待导航到达目标
        print("[蓝方] 武器等待超时，等待导航到达目标...")
        nav_thread.join()

    # ★ 关闭坐标取反, 后续恢复正常
    invert_coordinates = False
    sleep_check_retry(2)
    auto_go_to_target(target_x=0.994, target_y=1.551, target_yaw=0.0 )#点3
    auto_go_to_target(target_x=2.442, target_y=1.551, target_yaw=0.0 )#改为梅林中心点，x轴要改！！！！！！！！！！!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    # === 路径规划：根据地形配置自动生成指令 ===
    plan = plan_route("蓝方", config)
    if plan is None:
        print("[蓝方] 路径规划失败！")
        return
    execute_commands(plan["pre_cmds"])
    execute_commands(plan["main_cmds"])
    run_rotate_stage(0)
    auto_go_to_target(target_x=8.253,target_y=2.157, target_yaw=0.0 )
    auto_go_to_target(target_x=8.253,target_y=3.7, target_yaw=0.0 )
    set_step_mode(3)
    sleep_check_retry(2)
    auto_go_to_target(target_x=10.958,target_y=3.7, target_yaw=0.0 )
    set_step_mode(0)
    #下梅林点位(8.253,2.157)！！！！！！！！！！！!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    #上斜坡点位(10.958,3.7)!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    sleep_check_retry(100)
    print("========== 蓝方任务序列完成 ==========")


# ===================== 主程序 =====================
if __name__ == "__main__":
    # ── 0. 解析命令行参数 ──
    parser = argparse.ArgumentParser(description="RC2 自动任务脚本")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--red", action="store_true", help="红方模式")
    group.add_argument("--blue", action="store_true", help="蓝方模式")
    parser.add_argument("--no-ui", action="store_true",
                        help="跳过地形编辑器，直接使用已保存的 JSON 配置（若无则用默认）")
    args = parser.parse_args()

    if args.red:
        TEAM_COLOR = "红方"
        DEFAULT_CONFIG = RED_CONFIG_DEFAULT
    else:
        TEAM_COLOR = "蓝方"
        DEFAULT_CONFIG = BLUE_CONFIG_DEFAULT

    CONFIG_PATH = os.path.join(TERRAIN_CONFIG_DIR,
                               f"terrain_{'red' if args.red else 'blue'}.json")

    print(f"\n{'='*50}")
    print(f"  RC2 自动任务脚本 — {TEAM_COLOR}")
    print(f"{'='*50}")

    odom_node = None
    spin_thread = None
    send_thread = None
    read_thread = None

    try:
        # ── 1. 初始化串口 ──
        if not init_serial():
            exit(1)

        # ── 2. 启动串口接收线程 + 持续发送线程 ──
        read_thread = threading.Thread(target=serial_read_loop, daemon=True)
        read_thread.start()
        print("已启动串口接收线程")
        send_thread = threading.Thread(target=continuous_send_loop, daemon=True)
        send_thread.start()
        time.sleep(0.1)

        # ── 3. 启动 Livox + LIO  + ROS ──
        start_livox_driver()
        start_fastlio2()
        rclpy.init()
        odom_node = OdomSubscriber()
        def _spin_with_clean_exit(node):
            try:
                rclpy.spin(node)
            except rclpy.executors.ExternalShutdownException:
                pass  # ROS2 正常退出，无需打印 traceback

        spin_thread = threading.Thread(target=_spin_with_clean_exit, args=(odom_node,), daemon=True)
        spin_thread.start()
        print("已订阅 /fastlio2/lio_odom 里程计话题")

        # ── 3.5 启动里程计健康监控线程 ──
        #health_thread = threading.Thread(target=odom_health_monitor, daemon=True)
        #health_thread.start()

        # ── 4. 等待里程计稳定再执行任务 ──
        #if not wait_odom_healthy(stable_time=5.0):
        #    print("[启动检查] ✗ 里程计未能稳定，退出")
        #    exit(1)

        # ── 5. 加载地形配置 ──
        terrain_config = None

        if args.no_ui:
            print("\n[配置] --no-ui: 跳过编辑器，从文件加载...")
            terrain_config = load_config_from_file(CONFIG_PATH)
            if terrain_config is None:
                print(f"[配置] 文件不存在，使用默认配置")
                terrain_config = DEFAULT_CONFIG
            else:
                print(f"[配置] ✓ 已从 {CONFIG_PATH} 加载")
        else:
            print("\n[配置] 启动地形编辑器...")
            terrain_config = run_terrain_editor(TEAM_COLOR, CONFIG_PATH)
            if terrain_config is None:
                # 用户取消编辑器 → 尝试回退文件
                print("[配置] 编辑器已取消，尝试从文件加载...")
                terrain_config = load_config_from_file(CONFIG_PATH)
                if terrain_config is None:
                    print("[配置] 无已保存配置，使用默认")
                    terrain_config = DEFAULT_CONFIG
                else:
                    print(f"[配置] ✓ 已从 {CONFIG_PATH} 加载")

        t_cnt = sum(1 for v in terrain_config.values() if v == 't')
        f_cnt = sum(1 for v in terrain_config.values() if v == 'f')
        print(f"[配置] 最终地形: 目标={t_cnt}, 障碍={f_cnt}")

        # ── 6. 执行任务序列 ──
        if args.red:
            run_red_mission(terrain_config)
        else:
            run_blue_mission(terrain_config)

    except RetryInterrupt as e:
        retry_val = e.retry_val
        while retry_val:
            print(f"\n{'!'*60}")
            print(f"  检测到 RetryInterrupt: 0x{retry_val:02X}")
            print(f"  等待 {RETRY_WAIT_SEC}s 后执行恢复任务...")
            print(f"{'!'*60}")
            retry_disabled = True
            retry_triggered = 0
            cancel_nav = False
            stop_car()
            time.sleep(RETRY_WAIT_SEC)
            try:
                run_retry_mission(retry_val, TEAM_COLOR, terrain_config)
                retry_val = 0  # retry 任务正常完成，退出循环
            except RetryInterrupt as e2:
                retry_val = e2.retry_val  # 嵌套 retry: 继续循环处理新的 retry 指令

    except KeyboardInterrupt:
        print("\n检测到手动终止(Ctrl+C)")
    except Exception as e:
        print(f"\n程序异常: {e}")
    finally:
        clean_resources(odom_node, spin_thread, send_thread, read_thread)
