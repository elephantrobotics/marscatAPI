from pyfirmata import Arduino, util
import time

mars = Arduino('/dev/ttyUSB0')
#mars = Arduino('/dev/ttyUSB1')

import numpy as np

while True:
    input_data = input("Enter a number 1~66 or others for testing: ")
    # input_data = raw_input("Enter a number 1~5 or others for testing: ")

    # Base Mode
    if input_data == "10":
        mars.setMode(2, 0.2)
    elif input_data == "11":
        mode = mars.getMode(1) # 1 is version
        print(mode)
    elif input_data == "12":
        gyro_data = mars.getGyro(1)
        print(gyro_data)
    elif input_data == "13":
        tof_data = mars.getTof()
        print(tof_data)
    elif input_data == "14":
        battery = mars.getBattery()
        print(battery)
    elif input_data == "15":
        #data 1: servo_id
        #data 2: servo_state - 0- ping; 1- temperature; 2-move; 3-current; 4-load
        servo_state = mars.getServoData(1, 1)
        print(servo_state)
    elif input_data == "16":
        mars.setServoData(servo_id=11, data_id=40, l_data=128)
        time.sleep(1)

    elif input_data == "17":
        mars.enableServos(11, 0)

# Basic Control
    elif input_data == "20":
        mars.setHeadAngle(1, 20, 1)
        time.sleep(1)
        mars.setHeadAngle(1, -20, 1)
    elif input_data == "21":
        angle = mars.getHeadAngle(1)
        print(angle)
    elif input_data == "22":
        mars.setTailAngle(1, 20, 1)
    elif input_data == "23":
        angle = mars.getTailAngle(1)
        print(angle)
    elif input_data == "24":
        mars.setLegAngle(1, 1, 0, 1)
    elif input_data == "25":
        angle = mars.getLegAngle(1, 1)
        print(angle)

    elif input_data == "2A":
        mars.setLegOffset(1, 0.02, 0, 0, 1)

    elif input_data == "2B":
        mars.setCOGOffset(0.0, 0.0, 0.01, 0.5)

# Basic Move
    elif input_data == "30":
        speed = 1
        dx = 0.03
        mars.setTrot(1, dx, dy=0, speed=0.4, dtheta=-0,
                     steps=4) # step_num, dx, dy, speed, dtheta, steps
        time.sleep(4)
        mars.setTrot(2, dx, dy=0, speed=0.4, dtheta=-0,
                     steps=4) # step_num, dx, dy, speed, dtheta, steps
        time.sleep(4)
        mars.setTrot(2, dx, dy=0, speed=0.4, dtheta=-0,
                     steps=4) # step_num, dx, dy, speed, dtheta, steps
        time.sleep(4)
        mars.setTrot(0, dx, dy=0, speed=0.4, dtheta=-0,
                     steps=4) # step_num, dx, dy, speed, dtheta, steps
        time.sleep(4)
        #mars.setTrot(2,-dx,0,speed,0,16)

        mars.setStop()
    elif input_data == "31":

        mars.setTurn(10, 1, 1) # angle, speed, steps
        time.sleep(2)
        mars.setTurn(10, 0.7, 1) # angle, speed, steps
        time.sleep(2)
        mars.setTurn(10, 0.4, 1) # angle, speed, steps
        time.sleep(2)
        mars.setTurn(10, 0.1, 1) # angle, speed, steps
        time.sleep(2)

    elif input_data == "32":
        mars.setCrawl(step_num=3, speed=0.2, dx=0.04, dy=0, dtheta=0,
                      steps=2) #ste_num, speed, dx, dy, dtheta, steps
        time.sleep(5)
        mars.setCrawl(step_num=3, speed=0.7, dx=0.02, dy=0, dtheta=0,
                      steps=2) #ste_num, speed, dx, dy, dtheta, steps
        time.sleep(5)

        mars.setCrawl(step_num=2, speed=0.4, dx=0.04, dy=0, dtheta=0,
                      steps=2) #ste_num, speed, dx, dy, dtheta, steps
        time.sleep(5)
        mars.setCrawl(step_num=3, speed=0.4, dx=0.02, dy=0, dtheta=0,
                      steps=2) #ste_num, speed, dx, dy, dtheta, steps
        time.sleep(5)

    elif input_data == "3A":
        mars.setTimeDelay(45)
    elif input_data == "3B":
        time_delay = mars.getTimeDelay()
        print(time_delay)

# Calibration
    elif input_data == "40":
        mars.setCalibration()
    elif input_data == "41":
        cal_data = mars.getCalibration()
        print(cal_data)

# Test
    elif input_data == "50":
        mars.testBodyMove(1)
    elif input_data == "99":
        mars.setHeadAngle(2, -20, 1)
        time.sleep(1)
        start_time = time.time()
        a = []
        for i in range(-20, 21):
            #mars.setHeadAngle(2, i, 1)
            a.append(mars.getTof())
        print(a)
        print(time.time() - start_time)
    else:
        mars.setStop() # stop all
