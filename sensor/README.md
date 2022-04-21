<!--
Copyright (c) 2019 Elephant Robotics, Inc. All rights reserved.

Using this MarsAI source code is subject to the terms and conditions of Apache 2.0 License. Check LICENSE for more information
-->

# Touch

The touch is to get signals from the sensors inside MarsCat.

## Functions

### get_touch

- **Prototype**: `get_touch()`

- **Description**: Get signals from the sensors.

- **Return**
  - `GPIO.input() -> {value}`: Used to display value change of the IO


## Instructions

### Detect touch

```python
import sensor.touch
import time

tc = sensor.touch.Touch()

"""
If all sensors aren't triggered, it will return [0, 0, 0, 0, 0, 0, 0] 
If head sensors are triggered, it will return [1, 1, 0, 0, 0, 0, 0] 
If jaw sensors aren triggered, it will return [0, 0, 1, 0, 0, 0, 0] 
If back sensors are triggered, it will return [0, 0, 0, 1, 1, 1, 1] 
"""

while True:
    tc.get_touch()
    time.sleep(0.1)
```
