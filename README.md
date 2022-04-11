# MarsCat API

## Preparation before development
- Connect the monitor, mouse, and keyboard
- Or after connecting to wifi, use SSH or VNC for remote connection

> Note: The connected account and password are pi
- Before development, it is necessary to stop the startup service of the cat
    ```
    cd ~/marsai
    ./tools/stop-systemd-services.sh
    ./tools/disable-systemd-services.sh
    ```
- If you want the Marscat to move on its own
    ```
    cd ~/marsai
    ./tools/enable-systemd-services.sh
    ./tools/start-systemd-services.sh
    ```

## The basic motion
```
# file: move/movement.py
# See this file for more functions

import movement

mv = movement.MoveMent()
mv.set_walk(step = 2, speed = 0.7)
...
```

## Use of distance sensors
```
# file: move/movement.py
# See this file for more functions

import movement

mv = movement.MoveMent()
print(mv.get_tof())
...
```

## Get battery power
```
# file: move/movement.py
# See this file for more functions

import movement

mv = movement.MoveMent()
print(mv.get_battery())
...
```

## Touch detection
```
# file: sensor/touch.py

import touch

tc = touch.Touch()
tc.test_touch()

# or

import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
IOlist = [22, 23, 24, 16, 13, 21, 4]
res = [0 for _ in range(7)]
for i in range(len(IOlist)):
    GPIO.setup(IOlist[i], GPIO.IN)

while True:
    for i in range(len(IOlist)):
        res[i] = GPIO.input(IOlist[i])
    print(res)
```


## According to the eye
```
# file: eye/eyedisplay.py

import eyedisplay

eyes = eyedisplay.EyeDisplay()
eyes.display_eye(ball_num = 1, lid_num = 1, ball_type = 'eye_ball', dx = 0, dy = 0)

"""
args:
    ball_num --> int:
        normal eye_ball : 1~10 [e.g. eye_ball]
	eyes with animation : 1~3 [e.g. flip, hug, ball, etc]

        # Eyeball size
    lid_num --> int:
        1~14, 31~40
        # Eyelid size
    ball_type --> str:
        [eye_ball, ball, face, flip, heat, hug, teaser, voice,
                         low_power, charging, dizzy, bowlorhungry, sleep
                         high_temperature, hw_error]
        # To select which folder the picture comes from
        # eye/eyeball/
    dx,dy --> int:
        # Coordinates of eyeball
        # (0, 0) in the middle
"""
```
### Change eye image

Change the image of the eyeball

Add the new image (The image resolution is 128x128) to the eye/eyeball/eye_ball folder.

    For example: The new image added is called eye_ball_15.png

```python
import eyedisplay

eyes = eyedisplay.EyeDisplay()
eyes.display_eye(ball_num = 15)
```

## Play sound
```
# file: sound/catsound.py

import catsound

s = CatSound()
s.meow()
```

### Change the sound

1.If you want to add a new sound file, add it to the "Sounds" folder.One sound file corresponds to three files with different names.

    like this:
        sounds/new_file-quite.mp3
        sounds/new_file-medium.mp3
        sounds/new_file-loud.mp3
            
2.Change the catsound.py file
```python
# catsound.py
class CatSound():
    def __init__(self):
        pass
    ...
    def new_sound(self, async_play=True):
        self.play_sound('new_file.mp3', async_play)

    ...
```
3.Run
```
# file: sound/catsound.py

import catsound

s = CatSound()
s.new_sound()
```

## Image recognition
```
# file: vision/vision_base.py

import vision_base

vision = vision_base.Vision()
vision.test()
```
It can recognize faces, matching blue balls and QR codes.


## Speech recognition

```
# file: voice/voice_base.py

import voice_base
voice_base.test_voice()
```
### Select a speech recognition language
```
# file: voice/voice_base.py line 28 
def speak_config(self):
    language = "Chinese"
```

### Voice wake up

**Wake up the word**：
```
# file: voice/voice_base.py line 142

ACTIVATION_WORDS = [
                            'HI MARSCAT', 'MARSCAT', 'MASSCAT', 'MASKCAT', 'MARS',
                            'ASSCAT', 'MASS', '咪咪', '小猫', '猫'
                        ]
```

```
# file: voice/voice_base.py line 152
if ff:
    Wake up successful, enter listening mode
```
voice/corpus/***.dic The file contains all the identifiable words in the three languages