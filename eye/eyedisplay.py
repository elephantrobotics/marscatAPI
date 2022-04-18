import copy
import threading
import time
import sys
import os

sys.path.append(".")
from eye import OLED_Driver as OLED
from PIL import Image

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
        
    def blink(self, lid_percentage=1):
        if EyeDisplay.playing:
            return
        EyeDisplay.playing = True
        dx = self.current_eye[-1][0]
        dy = self.current_eye[-1][1]
        ball_percentage = self.current_eye[-1][2]
        for i in range(1, 12, 2):
            self.display_eye(ball_percentage, i, dx=dx, dy=dx)
        for i in range(1, 12, 2):
            self.display_eye(ball_percentage, 12 - i, dx=dx, dy=dy)
        time.sleep(0.1)
        del self.current_eye[:]
        self.current_eye.append((dx, dy, ball_percentage, lid_percentage))
        EyeDisplay.playing = False
        return self.current_eye

    def display_eye_squint(self, stop=False, random_times=2,
                           ball_percentage=8, lid_percentage=1):
        if EyeDisplay.playing:
            return
        EyeDisplay.playing = True
        dx = self.current_eye[-1][0]
        dy = self.current_eye[-1][1]
        sleep_time = 0.07
        for i in range(31, 41):
            self.display_eye(ball_percentage, i, dx=dx, dy=dy)

        for _ in range(random_times):
            for i in range(1, 5):
                self.display_eye(ball_percentage, 41 - i, dx=dx, dy=dy)
                time.sleep(0.07)
            for i in range(37, 41):
                self.display_eye(ball_percentage, i, dx=dx, dy=dy)
                time.sleep(sleep_time)
            sleep_time = 0.12
            if not stop:
                break

        time.sleep(0.1)
        for i in range(1, 11):
            self.display_eye(ball_percentage, 41 - i, dx=dx, dy=dy)
        self.display_eye(ball_percentage, 1, dx=dx, dy=dy)
        del self.current_eye[:]
        self.current_eye.append((dx, dy, ball_percentage, lid_percentage))
        EyeDisplay.playing = False
        return self.current_eye

    def display_eye_wakeUp(self, ball_num=8, lid_num=11, dx=0, dy=0):
        if EyeDisplay.playing:
            return
        EyeDisplay.playing = True
        self.display_eye(ball_num, lid_num, dx=dx, dy=dy)
        for b in range(1, 7):
            self.display_eye(ball_num, (11 - b), dx=dx, dy=dy)
            time.sleep(0.05)
        for b in range(6, 12):
            self.display_eye(ball_num, b, dx=dx, dy=dy)
        for b in range(1, 11):
            self.display_eye(ball_num, (11 - b), dx=dx, dy=dy)
            time.sleep(0.02)
        del self.current_eye[:]
        self.current_eye.append((dx, dy, ball_num, lid_num))
        EyeDisplay.playing = False
        return self.current_eye

    def display_eye_close(self, ball_percentage=8, lid_percentage=1):
        if EyeDisplay.playing:
            return
        EyeDisplay.playing = True
        dx = self.current_eye[-1][0]
        dy = self.current_eye[-1][1]
        ball_percentage = self.current_eye[-1][2]
        for l in range(1, 6):
            self.display_eye(ball_percentage, l * 2, dx=dx, dy=dy)
            time.sleep(0.08)
        EyeDisplay.playing = False

    def display_eye_sleepy(self, ball_percentage=8, lid_percentage=1):
        if EyeDisplay.playing:
            return
        EyeDisplay.playing = True
        dx = self.current_eye[-1][0]
        dy = self.current_eye[-1][1]
        for l in range(1, 9):
            self.display_eye(ball_percentage, l, dx=dx, dy=dy)
            time.sleep(0.01)
        for l in range(1, 5):
            self.display_eye(ball_percentage, 8 - l, dx=dx, dy=dy)
            time.sleep(0.1)
        for l in range(1, 8):
            self.display_eye(ball_percentage, l + 2, dx=dx, dy=dy)
            time.sleep(0.05)
        for l in range(1, 5):
            self.display_eye(ball_percentage, 9 - l, dx=dx, dy=dy)
            time.sleep(0.1)
        for l in range(1, 7):
            self.display_eye(ball_percentage, l + 5, dx=dx, dy=dy)
        EyeDisplay.playing = False

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
    a.display_eye_sleepy()
