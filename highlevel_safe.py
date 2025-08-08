import sys
import os
import time

import mc_sdk_py
import time
app=mc_sdk_py.HighLevel()
print("Initializing...")
app.initRobot("192.168.168.99",43988, "192.168.168.168") #local_ip, local_port, dog_ip
time.sleep(10)
print("Initialization completed")
def main():
    app.standUp()
    time.sleep(4)
    app.lieDown()
    time.sleep(4)
    app.standUp()
    time.sleep(4)
    # app.jump()
    # time.sleep(4)
    # app.frontJump()
    # time.sleep(4)
    # app.backflip()
    # time.sleep(4)
    #app.move(0.4, 0.0, 0.0)
    # app.attitudeControl(0.1,0.1,0.1,0.1)
    # time.sleep(4)
    # app.standUp()
    # time.sleep(4)
    # app.lieDown()
    # time.sleep(100)
    # app.passive()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        app.passive()
        time.sleep(2)