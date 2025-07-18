import sys
import os
import time

import mc_sdk_py
import time
app=mc_sdk_py.HighLevel()
print("Initializing...")
time.sleep(10)
app.initRobot("192.168.234.17",43988, "192.168.234.1") #local_ip, local_port, dog_ip
print("Initialization completed")
def main():
    app.standUp()
    time.sleep(4)
    app.lieDown()
    time.sleep(4)
    app.standUp()
    time.sleep(4)
    app.jump()
    time.sleep(4)
    app.frontJump()
    time.sleep(4)
    app.backflip()
    time.sleep(4)
    app.attitudeControl(0.1,0.1,0.1,0.1)
    time.sleep(4)
    app.standUp()
    time.sleep(4)
    app.passive()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        app.passive()
        time.sleep(2)