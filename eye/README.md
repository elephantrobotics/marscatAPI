<!--
Copyright (c) 2019 Elephant Robotics, Inc. All rights reserved.

Using this MarsAI source code is subject to the terms and conditions of Apache 2.0 License. Check LICENSE for more information
-->

# Eyedisplay

The eyedisplay is used to display pictures in the eyes of MarsCat.

## Functions

### reset_eye_background

- **Prototype**: `reset_eye_background(cls)`

- **Description**: Set eye background color.

- **Parameters**

  - `cls` : [Blue, Green, Purple, Yellow].


### blink

- **Prototype**: `blink()`

- **Description**: Display blink animation.

### display_eye_squint

- **Prototype**: `display_eye_squint()`

- **Description**: Display squint animation.

### display_eye_wakeUp

- **Prototype**: `display_eye_wakeUp()`

- **Description**: Display wake-up animation.

### display_eye_close

- **Prototype**: `display_eye_close()`

- **Description**: Display close eyes animation.

### display_eye_sleepy

- **Prototype**: `display_eye_sleepy()`

- **Description**: Display sleepy animation.

### get_eye_lid

- **Prototype**: `get_eye_lid(number)`

- **Description**: Convert eye lid images to RGBA images.

- **Parameters**

  - `number` : eye lid number 1 ~ 14 31 ~ 40.

### get_eye_ball

- **Prototype**: `get_eye_ball(number)`

- **Description**: Convert eye ball images to RGBA images.

- **Parameters**

  - `number` : eye ball number 1 ~ 10.

### display_eye

- **Prototype**: `display_eye(ball_num, lid_num, ball_type, dx, dy)`

- **Description**: Display specified eye animation.

- **Parameters**

  - `ball_num` : Eyeball size 1 ~ 10
  - `lid_num` : Eyelid size 1 ~ 14, 31 ~ 40 
  - `ball_type` : [eye_ball, ball,face,flip,heat,hug,teaser,voice,low_power,charging,dizzy, bowlorhungry, sleep,high_temperature, hw_error]
  - `dx,dy` : Coordinates of eyeball, (0, 0) means in the middle


## Instructions

### Display eye animations

```python

import eye.eyedisplay
import time

color = ['Blue', 'Green', 'Purple', 'Yellow']

eyes = eye.eyedisplay.EyeDisplay()

for c in color:
    eyes.reset_eye_background(c)
    time.sleep(1)

    eyes.blink()
    time.sleep(3)

    eyes.display_eye_squint()
    time.sleep(3)

    eyes.display_eye_wakeUp()
    time.sleep(3)

    eyes.display_eye_close()
    time.sleep(3)

    eyes.display_eye_sleepy()
    time.sleep(3)

    eyes.get_eye_lid(2)
    time.sleep(1)

    eyes.get_eye_ball(2)
    time.sleep(1)

    eyes.display_eye(ball_type ='charging')
    time.sleep(3)

```
