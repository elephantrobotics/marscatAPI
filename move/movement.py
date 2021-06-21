#!/usr/bin/python3
# coding=utf-8
# Author:xxx

import sys
import time
sys.path.append(".")

import subprocess
import library.pyFirmata.pyfirmata

class MoveMent():

    def __init__(self):
        _dev_index: str = subprocess.run(['echo -n /dev/ttyUSB*'],
                                     stdout=subprocess.PIPE,
                                     shell=True).stdout.decode('utf-8')

        self.mars = library.pyFirmata.pyfirmata.Arduino(_dev_index)

    # Basic Move
    def set_walk(self, step = 2, speed = 0.7):
        """
        args:
            step(int) : Number of steps to walk
            speed(float:0.1~1.0) : Speed of walking
        """
        self.mars.setCrawl(step_num=2, speed=speed, dx=0.03, dy=0, dtheta=0, steps=step)
        time.sleep(1.5*step)

    def set_run(self, step = 2, speed = 0.7):
        """
        args:
            step(int) : Number of steps to run
            speed(float:0.1~1.0) : Speed of running
        """
        self.mars.setTrot(step_num=2, dx=0.03, dy=0, speed=speed, dtheta=0, steps=step)
        time.sleep(0.8*step)

    def turn(self, direction='left', step=1, speed = 0.7):
        """
        args:
            direction(str:'left'or'right') : Direction of turning
            step(int) : Number of steps to turn
        """
        if direction == 'left':
            step_length = 5
        else:
            step_length = -5
        self.mars.setTrot(1, 0, 0, 0.8, step_length, 1)
        time.sleep(0.8)
        self.mars.setTrot(2, 0, 0, speed, step_length, step)
        time.sleep(0.8*step)
    
    def backward(self, step=5, speed=0.7):
        """
        args:
            step(int) : Number of steps to backward
            speed(float:0.1~1.0) : The speed of backward
        """
        self.mars.setTrot(2, -0.03, 0, speed, 0, step)
        time.sleep(1.2 * step)


    # Basic Control
    def set_head_angle(self, joint_no, angle, speed):
        '''
        args:
            joint_no(int)
                1 : up and down
                2 : left and right
            angle(int)
                1 : 0 ~ 40
                2 : -20 ~ +20
            speed(float)
                0~1
        '''
        self.mars.setHeadAngle(joint_no=joint_no, angle=angle, speed=speed)

    def get_head_angle(self, joint_no):
        '''
        args:
            joint_no
                1 : up and down
                2 : left and right  
        '''
        return self.mars.getHeadAngle(joint_no)
    
    def set_leg_angle(self, leg_no, joint_no, angle, speed):
        '''
        args:
            leg_no(int)
                1 ~ 4
            joint_no(int)
                1 ~ 3
            angle(int)
                -180 - 180
            speed(float)
                0 ~ 1
        '''
        self.mars.setLegAngle(leg_no=leg_no, joint_no=joint_no, angle=angle, speed=speed)
    
    def get_leg_angle(self, leg_no, joint_no):
        '''
        args:
            leg_no
                1 ~ 4
            joint_no
                1 ~ 3

        return 
            joint_angle
        '''
        return self.mars.getLegAngle(leg_no=leg_no, joint_no=joint_no)


    # Basic Mode
    def get_gyro(self, mode=1):
        '''
        Get gyroscope parameters
        args:
            mode(int):
                0/1-RX/RY 
                3/4/5-AX/AY/AZ

        return:
            gyro angle/ acceraltion 
        '''
        return self.mars.getGyro(mode)
    
    def get_tof(self):
        """
        distance detection
        return(float):
            tof_data
        """
        return self.mars.getTof()
    
    def get_battery(self, mode=1):
        """
        args:
            mode(int:0, 1):
                0 : get battery voltage
                1 : get battery percentage
        return:
            float
        """
        return self.mars.getBattery(mode)

    def set_servos_power(self, servo_no, servo_state):
        '''
        Electrify or energize a cat
        args:
            servo_no(int)
                0 : all servo
                1~16 : one servo 
            servo_state
                0 - power off  
                1 - power on 
                2 - foucs
        '''
        self.mars.enableServos(servo_no=servo_no, servo_state=servo_state)

    
    # Use Cases
# if __name__ == '__main__':
#     obj = MoveMent()
    # obj.set_head_angle(1,30,0.3)
    # time.sleep(2)
    # obj.set_head_angle(1,0,0.3)
    # time.sleep(2)
    # obj.set_head_angle(2,30,0.3)
    # time.sleep(2)
    # obj.set_head_angle(2,0,0.3)
    # print(obj.get_head_angle(1))
    # print(obj.get_head_angle(2))
