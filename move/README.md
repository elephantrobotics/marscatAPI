<!--
Copyright (c) 2019 Elephant Robotics, Inc. All rights reserved.

Using this MarsAI source code is subject to the terms and conditions of Apache 2.0 License. Check LICENSE for more information
-->

# Movement

The movement is used to control and get data from MarsCat.

## Functions

### set_walk

- **Prototype**: `set_walk(step, speed)`

- **Description**: Walk movement.

- **Parameters**

  - `step(int)` : Number of steps to walk.
  - `speed(float:0.1~1.0)` : Speed of walking

### set_run

- **Prototype**: `set_run(step, speed)`

- **Description**: Run movement.

- **Parameters**

  - `step(int)` : Number of steps to walk.
  - `speed(float:0.1~1.0)` : Speed of walking

### turn

- **Prototype**: `turn(direction, step, speed)`

- **Description**: Turn movement.

- **Parameters**

  - `direction(str:'left'or'right')` : Direction of turning
  - `step(int)` : Number of steps to walk.
  - `speed(float:0.1~1.0)` : Speed of walking

### backward

- **Prototype**: `backward(step, speed)`

- **Description**: Backward movement.

- **Parameters**

  - `step(int)` : Number of steps to walk.
  - `speed(float:0.1~1.0)` : Speed of walking

### set_head_angle

- **Prototype**: `set_head_angle(joint_no, angle, speed)`

- **Description**: Set head angle.

- **Parameters**

  - `joint_no(int)` :   1 - up and down, 2 - left and right
  - `angle(int)` : 1 - [4, 28], 2 - [-20, 20]
  - `speed(float:0.1~1.0)` : Speed of moving.

### get_head_angle

- **Prototype**: `get_head_angle(joint_no)`

- **Description**: Get head angle.

- **Parameters**

  - `joint_no(int)` :   1 - up and down, 2 - left and right

- **Return**

  - Joint_no angle.

### set_leg_angle

- **Prototype**: `set_walk(leg_no, joint_no, angle, speed)`

- **Description**: Walk movement.

- **Parameters**

  - `leg_no(int)` :   1 ~ 4
  - `joint_no(int)`: 1 ~ 3
  - `angle(int)` : 
    
    | leg | joint | min_angle | max_angle |
    | :----: | :----: |:----: |:----: |
    |leg 1 |joint 1     |  -20    |      20|
    |leg 1 |joint 2     |  -45    |      70|
    |leg 1| joint 3     |   5       |    75|
    |leg 2 |joint 1     |  -20   |       20|
    |leg 2 |joint 2     |  -45     |     70|
    |leg 2| joint 3     |   5      |     75|
    |leg 3 |joint 1     |  -20    |      20|
    |leg 3 |joint 2     |  -70    |      30|
    |leg 3 |joint 3     |  -100   |     -15|
    |leg 4| joint 1     |  -20     |     20|
    |leg 4 |joint 2     |  -70     |     30|
    |leg 4 |joint 3     |  -100  |      -15|

  - `speed(float:0.1~1.0)` : Speed of moving.

### get_leg_angle

- **Prototype**: `get_leg_angle(leg_no, joint_no)`

- **Description**: Get leg angle.

- **Parameters**

  - `log_no(int)` :   1 ~ 4
  - `joint_no(int)`: 1 ~ 3

- **Return**

  - Joint angle.

### get_gyro

- **Prototype**: `get_gyro(mode)`

- **Description**: Get gyro data.

- **Parameters**

  - `mode(int)` : 0/1-RX/RY 3/4/5-AX/AY/AZ

- **Return**

  - gyro angle/ acceraltion.

### get_tof

- **Prototype**: `get_tof()`

- **Description**: Get tof data.

- **Return**

  - tof_data(float).

### get_battery

- **Prototype**: `get_battery(mode)`

- **Description**: Get battery value.

- **Parameters**

  - `mode(int)` : 0 - get battery voltage , 1 - get battery percentage

- **Return**

  - battery_value(float).

### set_servos_power

- **Prototype**: `set_servos_power(servo_no, servo_state)`

- **Description**: Electrify or energize servo.

- **Parameters**

  - `servo_no(int)` : 0 - all servo , 1~16 - single servo
  - `servo_state` : 0 - power off, 1 - power on, 2 - foucs (servos can only be focused one by one)


## Instructions

### Use cases

```python
import move.movement
import time

mv = move.movement.MoveMent()

# Basic Move

mv.set_walk(8, 0.7)

mv.set_run(8, 0.7)

mv.turn('left', 10, 0.7)
mv.turn('right', 10, 0.7)

mv.backward(10, 0.7)

# Basic Control

for h in range(1, 3):
    if h == 1:
        for a in range(4, 28):
            mv.set_head_angle(h, a, 0)
    else:
        for a in range(-20, 20):
            mv.set_head_angle(h, a, 0)

for h in range(1, 3):
    print(mv.get_head_angle(h))

for l in range(1, 5):
    for j in range(1, 4):
        mv.set_leg_angle(l, j, 0, 0)

for l in range(1, 5):
    for j in range(1, 4):
        print(mv.get_leg_angle(l, j))

# Basic Mode

# 0 / 1 - RX / RY 3 / 4 / 5 - AX / AY / AZ
GR = [0, 1, 3, 4, 5]
for g in GR:
    mv.get_gyro(g)

mv.get_tof()

mv.get_battery(0)
mv.get_battery(1)

mv.set_servos_power(0, 1)
mv.set_servos_power(0, 0)
mv.set_servos_power(0, 2)
```
