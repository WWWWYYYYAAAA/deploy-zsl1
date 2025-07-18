import mc_sdk_py
import time
import numpy as np

motor_func = mc_sdk_py.LowLevel()
motor_func.initRobot("192.168.168.148", 43988, "192.168.168.168") #local_ip, local_port, dog_ip

while 1:
    if motor_func.haveMotorData():
        # state = motor_func.getMotorState()
        # print("q_abad:", state.q_abad)
        # print("q_hip:", state.q_hip)
        # print("q_knee:", state.q_knee)
        # print("q_foot:", state.q_foot)
        # print("v_abad:", state.v_abad)
        # print("v_hip:", state.v_hip)
        # print("v_knee:", state.v_knee)
        # print("v_foot:", state.v_foot)
        # print("timestep:", state.timestep)
        altitude = np.array((motor_func.getRoll(), motor_func.getPitch(), motor_func.getYaw()))
        print("Altitude:", altitude)
        Accel = np.array((motor_func.getBodyAccX(), motor_func.getBodyAccY(), motor_func.getBodyAccZ()))
        print("Accel:", Accel)

    else:
        print("No motor data available.")
        
    time.sleep(0.1)
        
        