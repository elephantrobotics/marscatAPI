import OLED_Driver as OLED
import copy
import threading
import random
import time
import sys
import os
import math
import json
from PIL import Image, ImageDraw, ImageFont

defalut_path = os.path.dirname(os.path.abspath(__file__))+"/eyeball"


class EyeDisplay():
    eye_background = None
    eye_color = ''
    eye_bg_image = None
    playing = False
    idle = threading.Event()

    def __init__(self):
        OLED.Device_Init()
        EyeDisplay.reset_eye_background()
        self.last_num = 0
        self.current_eye = [(0, 0, 8, 1)]  # (dx,dy,ball,lid)

    @classmethod
    def reset_eye_background(cls, color='Blue'):
        """
        set background color.
        color: [Blue, Green, Purple, Yellow]

        """
        if not cls.idle.is_set():
            cls.idle.set()
            cls.eye_color = color
            image_path = defalut_path
            print(image_path)
            cls.eye_background = Image.open(image_path + "/eye_pupil/" + "eye_" +
                                            EyeDisplay.eye_color + ".png").convert("RGBA")
            cls.eye_bg_image = Image.new(
                "RGBA", EyeDisplay.eye_background.size)
            cls.eye_bg_image.paste(EyeDisplay.eye_background, (0, 0),
                                   EyeDisplay.eye_background)
        else:
            cls.idle.wait()
        cls.idle.clear()
        # OLED.Device_Init()

    def get_eye_lid(self, number):
        number = int(number)
        image_name = "eyelid_" + str(number) + ".png"
        image_path = '/'.join([defalut_path, 'eye_lid', image_name])
        _eye_lid = Image.open(image_path).convert("RGBA")
        return _eye_lid

    def get_eye_ball(self, num, eye_type: str = 'eye_ball'):
        """
        Set up a picture of the eyeball
        """
        image_name = f'eye_ball_{num}.png' # get eye ball photo, You can change it to your image
        image_path = '/'.join([defalut_path, eye_type, image_name])
        _eye_ball = Image.open(image_path).convert("RGBA")
        print(image_path)
        return _eye_ball

    def display_eye(self, ball_num: int = 1, lid_num: int = 1,
                    ball_type: str = 'eye_ball',
                    dx: int = 0, dy: int = 0):
        """
        Args:
        ball_num : 
                normal eye_ball : 1 ~ 10 [e.g. eye_ball]
                eyes with animation : 1 ~ 3 [e.g. flip, hug, ball, etc]
        lid_num : 1 ~ 14, 31 ~ 40 
        dx,dy is coords of eyeball

        ball_type(str): [eye_ball, ball, face, flip, heat, hug, teaser, voice,
                         low_power, charging, dizzy, bowlorhungry, sleep
                         high_temperature, hw_error]
        """
        eye_raw = copy.deepcopy(self.eye_bg_image)
        eye_ball_raw = self.get_eye_ball(ball_num, eye_type=ball_type)
        eye_raw.paste(eye_ball_raw, (dx, dy), eye_ball_raw)
        eye_lid_raw = self.get_eye_lid(lid_num)
        eye_raw.paste(eye_lid_raw, (0, 0), eye_lid_raw)
        OLED.display_image(eye_raw)


if __name__ == '__main__':
    a = EyeDisplay()
    a.display_eye()
    time.sleep(1)
