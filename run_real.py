import time

# import mujoco.viewer
# import mujoco
import numpy as np
# from legged_gym import LEGGED_GYM_ROOT_DIR
import torch
import yaml
import mc_sdk_py
import time




def get_gravity_orientation(quaternion):
    qw = quaternion[0]
    qx = quaternion[1]
    qy = quaternion[2]
    qz = quaternion[3]

    gravity_orientation = np.zeros(3)

    gravity_orientation[0] = 2 * (-qz * qx + qw * qy)
    gravity_orientation[1] = -2 * (qz * qy + qw * qx)
    gravity_orientation[2] = 1 - 2 * (qw * qw + qz * qz)

    return gravity_orientation


def pd_control(target_q, q, kp, target_dq, dq, kd):
    """Calculates torques from position commands"""
    return (target_q - q) * kp + (target_dq - dq) * kd

# 定义全局变量 obs
obs = None

ctrl_f = [0,0] # ctrl_f key_f

def on_press(key):
    ctrl_f[1]=1
    if key == keyboard.Key.up:
        cmd[0] = 1.0
    elif key == keyboard.Key.down:
        cmd[0] = -1.0
    if key == keyboard.Key.left:
        if ctrl_f[0] == 0:
            cmd[1] = 1.0
        else:
            cmd[2] = 1.0
    elif key == keyboard.Key.right:
        if ctrl_f[0] == 0:
            cmd[1] = -1.0
        else:
            cmd[2] = -1.0
    if key == keyboard.Key.ctrl:
        ctrl_f[0] = 1

def on_release(key):
    ctrl_f[1]=0
    if key == keyboard.Key.up:
        cmd[0] = 0.0
    elif key == keyboard.Key.down:
        cmd[0] = 0.0
    if key == keyboard.Key.left:
        cmd[1] = 0.0
        cmd[2] = 0.0
    elif key == keyboard.Key.right:
        cmd[1] = 0.0
        cmd[2] = 0.0
    if key == keyboard.Key.ctrl:
        ctrl_f[0] = 0
        cmd[2] = 0.0

def padctrl():
    values = gamepad.GetInput(joyL=1,joyR=1,trigL=1,trigR=1,buttons=1,hat=1,joyL_max=100,os='linux')
    cmd[0] = 1.0*values[0][1]/100
    cmd[1] = -1.0*values[0][0]/100
    cmd[2] = 1.0*values[1][1]/100

def get_data_from_dog():
    acc_X = mc_sdk_py.get

class MotorControl:
    def __init__(self):
        self.motor_func = mc_sdk_py.LowLevel()
        self.motor_func.initRobot("192.168.168.148", 43988, "192.168.168.168") #local_ip, local_port, dog_ip
        self.init_q_abad = [0.0] * 4
        self.init_q_hip = [0.0] * 4
        self.init_q_knee = [0.0] * 4
        self.duration = 2.0
        self.default_abad_pos = 0.0
        self.default_hip_pos = 1.4
        self.default_knee_pos = -2.4
        self.stage1_progress = 0.0
        self.stage2_progress = 0.0
        self.stage = 0
        self.stage2_start = False
        self.first_trigger = False

    def run(self):
        while True:
            # 获取机器狗数据
            state = self.motor_func.getMotorState()

            if self.motor_func.haveMotorData():
                if not self.first_trigger:
                    self.first_trigger = True
                    for i in range(4):
                        self.init_q_abad[i] = state.q_abad[i]
                        self.init_q_hip[i] = state.q_hip[i]
                        self.init_q_knee[i] = state.q_knee[i]

                self.stage1_progress += 0.002
                ratio = self.stage1_progress / self.duration
                if ratio > 1.0:
                    ratio = 1.0
                    self.stage = 1
                
                if self.stage == 1:
                    self.default_abad_pos = 0.0
                    self.default_hip_pos = 0.8
                    self.default_knee_pos = -1.5
                    self.stage2_progress += 0.002
                    ratio = self.stage2_progress / self.duration
                    if ratio > 1.0:
                        ratio = 1.0
                    
                    if not self.stage2_start:
                        self.stage2_start = True
                        for i in range(4):
                            self.init_q_abad[i] = state.q_abad[i]
                            self.init_q_hip[i] = state.q_hip[i]
                            self.init_q_knee[i] = state.q_knee[i]

                cmd = mc_sdk_py.MotorCommand()
                for i in range(4):
                    cmd.q_des_abad[i] = ratio * self.default_abad_pos + (1.0 - ratio) * self.init_q_abad[i]
                    cmd.q_des_hip[i] = ratio * self.default_hip_pos + (1.0 - ratio) * self.init_q_hip[i]
                    cmd.q_des_knee[i] = ratio * self.default_knee_pos + (1.0 - ratio) * self.init_q_knee[i]
                    cmd.kp_abad[i] = 80
                    cmd.kp_hip[i] = 80
                    cmd.kp_knee[i] = 80
                    cmd.kd_abad[i] = 1
                    cmd.kd_hip[i] = 1
                    cmd.kd_knee[i] = 1

                ret = self.motor_func.sendMotorCmd(cmd)
                if ret < 0:
                    print("send cmd error")

            time.sleep(0.002)  # 等待 2 毫秒
    def stop(self):
        cmd = mc_sdk_py.MotorCommand()
        for i in range(4):
            cmd.q_des_abad[i] = 0.0
            cmd.q_des_hip[i] = 0.0
            cmd.q_des_knee[i] = -1.5
            cmd.kp_abad[i] = 0.0
            cmd.kp_hip[i] = 0.0
            cmd.kp_knee[i] = 0.0
            cmd.kd_abad[i] = 3.0
            cmd.kd_hip[i] = 3.0
            cmd.kd_knee[i] = 3.0
        flag = True
        send_msg_count = 0
        while flag:
            ret = self.motor_func.sendMotorCmd(cmd)
            if ret < 0:
                print("send cmd error")
            send_msg_count += 1
            if send_msg_count > 1500:
                flag = False
            time.sleep(0.002)  # 等待 2 毫秒

if __name__ == "__main__":
    config_file = "zsl1_real.yaml"
    with open(f"./config/{config_file}", "r") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
        policy_path = config["policy_path"]

        # simulation_duration = config["simulation_duration"]
        # simulation_dt = config["simulation_dt"]
        # control_decimation = config["control_decimation"]

        kps = np.array(config["kps"], dtype=np.float32)
        kds = np.array(config["kds"], dtype=np.float32)

        default_angles = np.array(config["default_angles"], dtype=np.float32)

        lin_vel_scale = config["lin_vel_scale"]
        ang_vel_scale = config["ang_vel_scale"]
        dof_pos_scale = config["dof_pos_scale"]
        dof_vel_scale = config["dof_vel_scale"]
        action_scale = config["action_scale"]
        cmd_scale = np.array(config["cmd_scale"], dtype=np.float32)

        num_actions = config["num_actions"]
        num_obs = config["num_obs"]
        num_one_step_obs = config["num_one_step_obs"]
        
        # cmd = np.array(config["cmd_init"], dtype=np.float32)

    cmd = np.array([0,0,0])
    # define context variables
    action = np.zeros(num_actions, dtype=np.float32)
    target_dof_pos = default_angles.copy()
    obs = np.zeros(num_obs, dtype=np.float32)
    current_obs = np.zeros(num_one_step_obs, dtype=np.float32)

    #lowlevel mc
    print("Initializing...")
    motor_control = MotorControl()
    time.sleep(10)
    print("Initialization completed")

    policy = torch.jit.load(policy_path)

    #ctrl
    from pynput import keyboard

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    from F710GamePad import F710GamePad

    gamepad = F710GamePad()
    emergency_stop = 1
    while emergency_stop:
        
        if ctrl_f[1] == 0:
            padctrl()
        # print(current_obs[:3] )
        current_obs[:3] = cmd * cmd_scale
        current_obs[3:6] = ang_vel
        current_obs[6:9] = gravity_orientation
        current_obs[9 : 9 + num_actions] = qj
        current_obs[9 + num_actions : 9 + 2 * num_actions] = dqj
        current_obs[9 + 2 * num_actions : 9 + 3 * num_actions] = action
        
        # 将当前观测数据添加到 obs 的开头，并将历史数据向前移动
        obs = np.concatenate((current_obs, obs[:-num_one_step_obs]))
        
        obs_tensor = torch.from_numpy(obs).unsqueeze(0)
        # policy inference
        action = policy(obs_tensor).detach().numpy().squeeze()
        target_dof_pos = action * action_scale + default_angles
     