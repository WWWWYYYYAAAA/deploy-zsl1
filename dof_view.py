import mc_sdk_py
import numpy as np
import time
import math

motor_func = mc_sdk_py.LowLevel()
motor_func.initRobot("192.168.168.99", 43988, "192.168.168.168") #local_ip, local_port, dog_ip

def get_data_from_dog():
    ang_vel = np.array((motor_func.getBodyGyroX(), motor_func.getBodyGyroY(), motor_func.getBodyGyroZ()))
    gravity_orientation = np.array((motor_func.getBodyAccX(), motor_func.getBodyAccY(), motor_func.getBodyAccZ()))*-1.0
    motorstate = motor_func.getMotorState()
    q_abad = np.array(motorstate.q_abad)
    q_hip = np.array(motorstate.q_hip)
    q_knee = np.array(motorstate.q_knee)
    
    qj = np.column_stack((q_abad, q_hip, q_knee)).flatten()
    dq_abad = np.array(motorstate.qd_abad)
    dq_hip = np.array(motorstate.qd_hip)
    dq_knee = np.array(motorstate.qd_knee)
    print(dq_abad)
    dqj = np.column_stack((dq_abad, dq_hip, dq_knee)).flatten()
    
    return ang_vel, gravity_orientation, qj, dqj

def get_data_from_dog2():
        ang_vel = np.array((motor_func.getBodyGyroX(), motor_func.getBodyGyroY(), motor_func.getBodyGyroZ()))
        gravity_orientation = np.array((motor_func.getBodyAccX(), motor_func.getBodyAccY(), motor_func.getBodyAccZ()))*-1.0
        motorstate = motor_func.getMotorState()
        q_abad = np.array(motorstate.q_abad)
        q_hip = np.array(motorstate.q_hip)
        q_knee = np.array(motorstate.q_knee)
        qj = np.column_stack((q_abad, q_hip, q_knee)).flatten()[[3,4,5,0,1,2,9,10,11,6,7,8]]
        dq_abad = np.array(motorstate.qd_abad)
        dq_hip = np.array(motorstate.qd_hip)
        dq_knee = np.array(motorstate.qd_knee)
        dqj = np.column_stack((dq_abad, dq_hip, dq_knee)).flatten()[[3,4,5,0,1,2,9,10,11,6,7,8]]
        
        return ang_vel, gravity_orientation, qj, dqj

while 1:
    if motor_func.haveMotorData():
        print("###############################################")
        ang_vel, gravity_orientation, qj, dqj = get_data_from_dog2()
        print("###############################################")
        # print("Angular Velocity:", ang_vel)
        # print("Gravity Orientation:", gravity_orientation)
        print("Joint Positions:", qj)
        # print("Joint Velocities:", dqj)
        # roll = motor_func.getRoll()
        # pitch = motor_func.getPitch()
        # gravity_orientation = np.array([-math.sin(pitch), math.sin(roll) * math.cos(pitch), -math.cos(roll) * math.cos(pitch)])
        # print(gravity_orientation)
        time.sleep(0.1)
    else:
        print("No motor data available.")
        
    time.sleep(0.1)
        
