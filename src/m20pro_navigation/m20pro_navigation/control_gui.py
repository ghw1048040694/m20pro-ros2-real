import socket
import json
import struct
import tkinter as tk
from tkinter import ttk, scrolledtext
import time
import threading

ROBOT_IP = '10.21.31.103'
ROBOT_PORT = 30001

# ---------------- 网络 ----------------
def create_connection(ip, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((ip, port))
    return s

def send_tcp(sock, obj):
    body = json.dumps(obj, separators=(',', ':')).encode()
    hdr = b'\xeb\x91\xeb\x90' + struct.pack('<H', len(body)) + \
          struct.pack('<H', 0) + b'\x01' + b'\x00'*7
    sock.sendall(hdr + body)

# ---------------- 业务功能 ----------------
def send_navigation_task(sock, pos_x, pos_y, pos_z, angle_yaw, point_info, gait, speed, manner, obs_mode, nav_mode, out_lbl):
    msg = {
        "PatrolDevice": {
            "Type": 1003, "Command": 1,
            "Time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "Items": {
                "Value": 1, "MapID": 0,
                "PosX": pos_x, "PosY": pos_y, "PosZ": pos_z,
                "AngleYaw": angle_yaw, "PointInfo": point_info,
                "Gait": gait, "Speed": speed, "Manner": manner,
                "ObsMode": obs_mode, "NavMode": nav_mode
            }
        }
    }
    send_tcp(sock, msg)
    rsp = sock.recv(4096)
    if not rsp:
        out_lbl.config(text="无响应"); return
    try:
        d = json.loads(rsp[16:].decode())['PatrolDevice']['Items']
        val, sts, err = d['Value'], d['Status'], d['ErrorCode']
        status_txt = {0:"空闲",1:"退出充电桩中",2:"导航预处理",3:"导航中",4:"导航完成",
                      5:"进入充电桩中",0xff:"暂停中"}.get(sts,f"未知状态({sts})")
        err_txt = "成功" if err == 0 else f"失败  ErrorCode=0x{err:04X}"
        out_lbl.config(
            text=f"下发结果：{err_txt}\n\n"
                 f"Value（目标点编号）：{val}\n"
                 f"Status（执行状态）：{status_txt}\n"
                 f"ErrorCode：0x{err:04X}"
        )
    except Exception as e:
        out_lbl.config(text=f"解析异常: {e}")

def cancel_navigation_task(sock, lbl):
    msg = {"PatrolDevice": {"Type": 1004, "Command": 1, "Time": time.strftime("%Y-%m-%d %H:%M:%S"), "Items": {}}}
    send_tcp(sock, msg)
    rsp = sock.recv(4096)
    if not rsp:
        lbl.config(text="取消无响应"); return
    try:
        err = json.loads(rsp[16:].decode())['PatrolDevice']['Items']['ErrorCode']
        lbl.config(text=f"取消导航任务 {'成功' if err == 0 else f'失败  ErrorCode=0x{err:04X}'}")
    except Exception as e:
        lbl.config(text=f"取消异常: {e}")

def get_navigation_status(sock, lbl):
    msg = {"PatrolDevice": {"Type": 2002, "Command": 1, "Time": time.strftime("%Y-%m-%d %H:%M:%S"), "Items": {}}}
    send_tcp(sock, msg)
    rsp = sock.recv(4096)
    if not rsp:
        lbl.config(text="无响应"); return
    try:
        st = json.loads(rsp[16:].decode())
        loc = st['PatrolDevice']['Items']['Location']
        obs = st['PatrolDevice']['Items']['ObsState']
        loc_txt = "定位正常" if loc == 0 else "定位丢失"
        obs_txt = "无障碍物" if obs == 0 else "有障碍物"
        lbl.config(text=f"Location: {loc} ({loc_txt})\nObsState: {obs} ({obs_txt})")
    except Exception as e:
        lbl.config(text=f"异常: {e}")

def initialize_and_reset_location(sock, lbl):
    msg = {"PatrolDevice": {"Type": 2101, "Command": 1, "Time": time.strftime("%Y-%m-%d %H:%M:%S"),
                            "Items": {"PosX": 0.0, "PosY": 0.0, "PosZ": 0.0, "Yaw": 0.0}}}
    send_tcp(sock, msg)
    rsp = sock.recv(4096)
    if not rsp:
        lbl.config(text="初始化无响应"); return
    try:
        err = json.loads(rsp[16:].decode())['PatrolDevice']['Items']['ErrorCode']
        lbl.config(text="初始化和重置定位成功" if err == 0 else "初始化和重置定位失败")
    except Exception as e:
        lbl.config(text=f"初始化异常: {e}")

def get_map_position(sock, lbl):
    msg = {"PatrolDevice": {"Type": 1007, "Command": 2, "Time": time.strftime("%Y-%m-%d %H:%M:%S"), "Items": {}}}
    send_tcp(sock, msg)
    rsp = sock.recv(4096)
    if not rsp:
        lbl.config(text="位置无响应"); return
    try:
        d = json.loads(rsp[16:].decode())['PatrolDevice']['Items']
        loc, loc_txt = d['Location'], ("定位正常" if d['Location'] == 0 else "定位丢失")
        x, y, z = d['PosX'], d['PosY'], d['PosZ']
        roll, pitch, yaw = d['Roll'], d['Pitch'], d['Yaw']
        lbl.config(text=f"Location: {loc} ({loc_txt})\n"
                        f"PosX: {x:.3f}\nPosY: {y:.3f}\nPosZ: {z:.3f}\n"
                        f"Roll: {roll:.3f}\nPitch: {pitch:.3f}\nYaw: {yaw:.3f}")
    except Exception as e:
        lbl.config(text=f"位置异常: {e}")

# ---------------- 运动状态转换功能（改为选择框模式） ----------------
def send_motion_state(sock, motion_param, motion_desc, lbl):
    """发送运动状态转换指令，参数来自选择框选择的运动状态"""
    # 构造运动状态转换指令（按《山猫M20系列软件开发手册V0.0.9.pdf》1.2.3节协议格式）
    msg = {
        "PatrolDevice": {
            "Type": 2, "Command": 22,  # Type=2, Command=22
            "Time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "Items": {
                "MotionParam": motion_param  # 运动状态参数
            }
        }
    }
    
    try:
        send_tcp(sock, msg)
        # 接收响应，增加超时处理（手册未明确部分状态是否返回响应，兼容无响应场景）
        sock.settimeout(2)
        rsp = sock.recv(4096)
        sock.settimeout(None)
        
        if not rsp:
            lbl.config(text=f"已发送运动状态指令：{motion_desc}\n（无响应，建议通过「获取导航状态」确认执行结果）")
        else:
            # 解析响应（若有），按手册协议格式提取数据
            try:
                rsp_data = json.loads(rsp[16:].decode())
                if "Items" in rsp_data.get("PatrolDevice", {}):
                    lbl.config(text=f"运动状态转换成功！\n选择状态：{motion_desc}\n响应数据：{rsp_data['PatrolDevice']['Items']}")
                else:
                    lbl.config(text=f"运动状态转换指令已发送！\n选择状态：{motion_desc}")
            except Exception as e:
                lbl.config(text=f"运动状态转换指令已发送！\n选择状态：{motion_desc}\n响应解析异常：{str(e)}")
    except socket.timeout:
        lbl.config(text=f"发送运动状态指令超时：{motion_desc}\n（请检查机器人TCP连接：{ROBOT_IP}:{ROBOT_PORT}）")
    except Exception as e:
        lbl.config(text=f"发送运动状态指令失败：{str(e)}\n选择状态：{motion_desc}")

# ---------------- WASD 轴指令 ----------------
axis_state = {'X': 0.0, 'Y': 0.0, 'Yaw': 0.0}
speed_scale = 1.0

def send_axis(sock):
    if sock is None: return
    # 轴指令格式按《山猫M20系列软件开发手册V0.0.9.pdf》1.2.5节定义
    msg = {
        "PatrolDevice": {
            "Type": 2, "Command": 21,
            "Time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "Items": {
                "X": axis_state['X'] * speed_scale,
                "Y": axis_state['Y'] * speed_scale,
                "Z": 0.0,
                "Roll": 0.0,
                "Pitch": 0.0,
                "Yaw": axis_state['Yaw'] * speed_scale
            }
        }
    }
    send_tcp(sock, msg)

def axis_timer(sock):
    while True:
        send_axis(sock)
        time.sleep(0.05)  # 按手册1.2.5节建议，轴指令发送频率20Hz（间隔0.05s）

def on_key_press(event, sock):
    global axis_state
    key = event.keysym.lower()
    if key == 'w': axis_state['X'] = 1.0
    elif key == 's': axis_state['X'] = -1.0
    elif key == 'a': axis_state['Y'] = 1.0
    elif key == 'd': axis_state['Y'] = -1.0
    elif key == 'q': axis_state['Yaw'] = 1.0
    elif key == 'e': axis_state['Yaw'] = -1.0
    elif key == 'space':
        axis_state = {'X': 0.0, 'Y': 0.0, 'Yaw': 0.0}
        send_axis(sock)

def on_key_release(event, sock):
    global axis_state
    key = event.keysym.lower()
    if key in ('w', 's'): axis_state['X'] = 0.0
    elif key in ('a', 'd'): axis_state['Y'] = 0.0
    elif key in ('q', 'e'): axis_state['Yaw'] = 0.0

# ---------------- 界面（滚动条+运动状态选择框） ----------------
def create_gui(sock):
    root = tk.Tk()
    root.title("山猫M20Pro 控制台")
    root.configure(bg='white')
    root.geometry("800x600")

    style = ttk.Style(root)
    style.theme_use('clam')
    style.configure('TLabel',  background='white', foreground='black', font=('Arial', 10))
    style.configure('TButton', background='#f0f0f0', foreground='black', font=('Arial', 10), borderwidth=1)
    style.map('TButton', background=[('active', '#e0e0e0')])
    style.configure('TEntry', fieldbackground='white', foreground='black', insertcolor='black')
    style.configure('TCombobox', fieldbackground='white', foreground='black', font=('Arial', 10))

    # ---------------- 滚动容器（保留） ----------------
    main_container = ttk.Frame(root)
    main_container.pack(fill='both', expand=True, padx=10, pady=10)
    
    v_scrollbar = ttk.Scrollbar(main_container, orient='vertical')
    v_scrollbar.pack(side='right', fill='y')
    
    canvas = tk.Canvas(main_container, yscrollcommand=v_scrollbar.set, bg='white', highlightthickness=0)
    canvas.pack(side='left', fill='both', expand=True)
    
    v_scrollbar.config(command=canvas.yview)
    
    main = ttk.Frame(canvas, padding=12)
    canvas.create_window((0, 0), window=main, anchor='nw')
    
    def update_scroll_region(event):
        canvas.configure(scrollregion=canvas.bbox('all'))
    main.bind('<Configure>', update_scroll_region)
    
    def on_mouse_wheel(event):
        canvas.yview_scroll(-int(event.delta/120), 'units')
    canvas.bind_all('<MouseWheel>', on_mouse_wheel)

    # ---------------- 运动状态转换控制区域（改为选择框） ----------------
    motion_frm = ttk.LabelFrame(main, text="运动状态转换", padding=10)
    motion_frm.pack(fill='x', pady=5)
    
    # 运动状态选项：严格按《山猫M20系列软件开发手册V0.0.9.pdf》1.2.3节定义的参数值与含义
    motion_options = [
        ("空闲", 0),
        ("站立", 1),
        ("关节阻尼/软急停", 2),
        ("开机阻尼", 3),
        ("趴下", 4),
    ]
    # 转换为选择框的显示文本列表
    motion_display = [opt[0] for opt in motion_options]
    # 选择框变量
    motion_var = tk.StringVar(value=motion_display[0])
    
    # 1. 选择框标签
    ttk.Label(motion_frm, text="选择运动状态：", width=15, anchor='e').grid(row=0, column=0, sticky='e', pady=5)
    # 2. 运动状态选择框
    motion_combobox = ttk.Combobox(
        motion_frm,
        textvariable=motion_var,
        values=motion_display,
        state='readonly',  # 仅允许选择，禁止手动输入
        width=20
    )
    motion_combobox.grid(row=0, column=1, sticky='w', pady=5, padx=5)
    # 3. 发送按钮
    ttk.Button(
        motion_frm,
        text="发送运动状态指令",
        command=lambda: send_motion_state(
            sock,
            # 根据选择的显示文本获取对应的参数值
            [opt[1] for opt in motion_options if opt[0] == motion_var.get()][0],
            motion_var.get(),
            motion_feedback_lbl
        )
    ).grid(row=0, column=2, pady=5, padx=5)
    
    # 4. 指令反馈标签
    motion_feedback_lbl = ttk.Label(
        motion_frm, 
        text="提示：选择运动状态后点击「发送运动状态指令」执行操作", 
        justify='left', 
        foreground='#27ae60'
    )
    motion_feedback_lbl.grid(row=1, column=0, columnspan=3, sticky='w', pady=3)

       # ---------------- 导航任务参数区域 ----------------
    in_frm = ttk.LabelFrame(main, text="导航任务参数", padding=10)
    in_frm.pack(fill='x', pady=5)
    labs = ["X坐标(m)", "Y坐标(m)", "Z坐标(m)", "朝向Yaw(rad)", "目标点类型", "步态", "速度", "运动方式", "避障模式", "导航模式"]
    ents = []
    for i, lab in enumerate(labs):
        ttk.Label(in_frm, text=lab, width=12, anchor='e').grid(row=i, column=0, sticky='e', pady=2)
        ent = ttk.Entry(in_frm, width=18)
        # 预设默认值（新版 gait 码）
        if i == 4: ent.insert(0, "0")        # 目标点类型：过渡点=0
        elif i == 5: ent.insert(0, "0x3002") # 步态：平地(敏捷运动模式)=0x3002
        elif i == 6: ent.insert(0, "0")      # 速度：正常=0
        elif i == 7: ent.insert(0, "0")      # 运动方式：前进行走=0
        elif i == 8: ent.insert(0, "0")      # 避障模式：开启=0
        elif i == 9: ent.insert(0, "0")      # 导航模式：直线导航=0
        ent.grid(row=i, column=1, pady=2)
        ents.append(ent)

    # ---------------- 导航任务按钮区域 ----------------
    btn_frm = ttk.Frame(main)
    btn_frm.pack(pady=8)
    ttk.Button(btn_frm, text="下发导航任务", width=22,
           command=lambda: send_navigation_task(
               sock,
               *[float(e.get() or 0) if i < 4 else int(e.get() or 0, 0) for i, e in enumerate(ents)],
               out_lbl=result_lbl)).grid(row=0, column=0, padx=5)
    ttk.Button(btn_frm, text="取消导航任务", width=22,
               command=lambda: cancel_navigation_task(sock, status_lbl)).grid(row=0, column=1, padx=5)

    # ---------------- 导航结果显示区域 ----------------
    result_lbl = ttk.Label(main, text="点击「下发导航任务」后显示返回参数", justify='left', foreground='#d35400')
    result_lbl.pack(pady=5)

    # ---------------- 轴指令控制区域 ----------------
    axis_frm = ttk.LabelFrame(main, text="轴指令控制（WASD移动/QE旋转/空格急停）", padding=10)
    axis_frm.pack(fill='x', pady=5)
    ttk.Label(axis_frm, text="速度倍率：").grid(row=0, column=0, sticky='e')

    speed_slider = tk.Scale(axis_frm, from_=0, to=1, resolution=0.1, orient='horizontal',
                            command=lambda v: globals().__setitem__('speed_scale', float(v)),
                            bg='white', troughcolor='#f0f0f0', highlightthickness=0)
    speed_slider.set(1.0)
    speed_slider.grid(row=0, column=1, sticky='we', padx=5)
    ttk.Label(axis_frm, text="轴指令发送频率20Hz，基础/楼梯步态仅X/Y/Yaw生效", font=('Arial', 9)).grid(row=1, column=0, columnspan=2, pady=5)

    # ---------------- 状态与位置区域 ----------------
    stat_frm = ttk.LabelFrame(main, text="状态与位置查询", padding=10)
    stat_frm.pack(fill='both', expand=True, pady=6)
    status_lbl = ttk.Label(stat_frm, text="点击「获取导航状态」显示定位与避障信息", justify='left')
    status_lbl.pack(anchor='w')
    pos_lbl = ttk.Label(stat_frm, text="点击「获取地图位置」显示地图坐标系坐标", justify='left')
    pos_lbl.pack(anchor='w', pady=(8,0))

    btn2_frm = ttk.Frame(stat_frm)
    btn2_frm.pack(pady=8)
    ttk.Button(btn2_frm, text="获取导航状态（1.4.3）", width=18,
               command=lambda: get_navigation_status(sock, status_lbl)).grid(row=0, column=0, padx=4)
    ttk.Button(btn2_frm, text="初始化/重置定位（1.4.1）", width=18,
               command=lambda: initialize_and_reset_location(sock, status_lbl)).grid(row=0, column=1, padx=4)
    ttk.Button(btn2_frm, text="获取地图位置（1.4.2）", width=18,
               command=lambda: get_map_position(sock, pos_lbl)).grid(row=0, column=2, padx=4)

    # ---------------- 键盘绑定（仅保留轴指令相关） ----------------
    root.bind('<KeyPress>', lambda e: on_key_press(e, sock))
    root.bind('<KeyRelease>', lambda e: on_key_release(e, sock))
    root.focus_set()

    # ---------------- 20Hz 轴指令发送线程 ----------------
    threading.Thread(target=axis_timer, args=(sock,), daemon=True).start()

    root.mainloop()

def main():
    # 建立TCP连接（按手册1.1.2节定义的TCP端口30001）
    sock = create_connection(ROBOT_IP, ROBOT_PORT)
    create_gui(sock)


if __name__ == '__main__':
    main()
