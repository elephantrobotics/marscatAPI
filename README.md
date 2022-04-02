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

## Touch detection
```
# file: sensor/touch.py

import touch

tc = touch.Touch()
tc.test_touch()
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

> If you want to use the new eye image, check the code for eye/eyedisplay.py

## Play sound
```
# file: sound/catsound.py

import catsound

s = CatSound()
s.meow()
```
> If you want to change the sound, check the code for sound/catsound.py

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