import sys
import os
import time
import mc_sdk_py
from pynput import keyboard  # 引入键盘监听库

# 初始化机器狗
app = mc_sdk_py.HighLevel()
print("Initializing...")
app.initRobot("192.168.168.148", 43988, "192.168.168.168")  # local_ip, local_port, dog_ip
time.sleep(5)
print("Initialization completed")

# 全局运动参数
vx, vy, yaw_rate = 0.0, 0.0, 0.0
MOVE_SPEED = 0.4  # 默认速度（m/s）
YAW_SPEED = 0.3   # 默认旋转速度（rad/s）

def on_press(key):
    global vx, vy, yaw_rate
    try:
        # 处理字符键（统一小写判断）
        if key.char == 'w':         # 前进
            vx = MOVE_SPEED
        elif key.char == 's':       # 后退
            vx = -MOVE_SPEED
        elif key.char == 'q':       # 左转
            yaw_rate = YAW_SPEED
        elif key.char == 'e':       # 右转
            yaw_rate = -YAW_SPEED
        elif key.char == 'd':       # 右平移
            vy = -MOVE_SPEED
        elif key.char == 'a':       # 左平移
            vy = MOVE_SPEED
    except AttributeError:
        # 处理特殊键（如空格、方向键等）
        if key == keyboard.Key.space:  # 急停
            vx, vy, yaw_rate = 0.0, 0.0, 0.0
  
    except AttributeError:
        pass  # 忽略特殊键异常

def on_release(key):
    """处理按键释放事件（停止运动）"""
    global vx, vy, yaw_rate
    if key in [keyboard.Key.up, keyboard.Key.down]:
        vx = 0.0
    elif key in [keyboard.Key.left, keyboard.Key.right]:
        yaw_rate = 0.0
    if key == keyboard.Key.esc:  # 按ESC退出监听
        return False

def main():
    app.standUp()
    time.sleep(2)
    
    # 启动键盘监听
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    
    app.standUp()
    time.sleep(2)
    app.lieDown()
    time.sleep(2)
    app.standUp()
    time.sleep(2)
    try:
        while True:
            # 持续发送运动指令
            ret = app.move(vx, vy, yaw_rate)
            if ret != 0:
                print(f"运动异常！错误码: {hex(ret)}")
            time.sleep(0.1)  # 控制指令发送频率
    except KeyboardInterrupt:
        pass
    finally:
        # 程序退出时复位
        app.move(0, 0, 0)
        app.lieDown()
        time.sleep(1)
        app.passive()

if __name__ == "__main__":
    main()