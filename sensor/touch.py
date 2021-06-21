#!/usr/bin/python3
# coding=utf-8
# Author:xxx

import RPi.GPIO as GPIO
import time


class Touch():

    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        data = {}
        self.IOlist = [22, 23, 24, 16, 13, 21, 4]
        for i in range(len(self.IOlist)):
            GPIO.setup(self.IOlist[i], GPIO.IN)

        self.counter_list = [0 for _ in range(len(self.IOlist))]
        # init get GPIO data, check
        self.ignore = [0 for _ in range(len(self.IOlist))]
        for i in range(len(self.IOlist)):
            p = GPIO.input(self.IOlist[i])
            self.ignore[i] = p
        print(f'touch ignore = {self.ignore}')


    def get_touch(self):
        value = [0 for _ in range(len(self.IOlist))]
        for i in range(len(self.IOlist)):
            aa = GPIO.input(self.IOlist[i])
            value[i] = aa

            # record 1 consecutive occurences.
            if aa == 1:
                self.counter_list[i] += 1
            else:
                self.counter_list[i] = 0

            # reopen the port
            if aa == 0 and self.ignore[i] == 1:
                self.ignore[i] = 0
            # if some position data not right, ignore it.
            if self.ignore[i] == 1:
                value[i] = 0

        # check counter list, close exception port.
        for idx, _v in enumerate(self.counter_list):
            if _v >= 50:
                self.ignore[idx] = 1

        print(f"GPIO.input() -> {value}")
        return value


    def test_touch(self):
        while 1:
            print(self.get_touch())
            time.sleep(0.1)


if __name__ == '__main__':
    tc = Touch()
    tc.test_touch()
