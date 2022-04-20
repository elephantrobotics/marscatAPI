<!--
Copyright (c) 2019 Elephant Robotics, Inc. All rights reserved.

Using this MarsAI source code is subject to the terms and conditions of Apache 2.0 License. Check LICENSE for more information
-->

# Vision

The vision is for using the camera on the nose to detect fall people, detect objects, detect faces or checking moving objects.

## Functions

### process_frame

- **Prototype**: `process_frame(frame)`

- **Description**: Parse video frames.

- **Parameters**

  - `frme` : video frame.

- **Return**
  - `corners`: Used to identify the specified QR code.
  - `ids`: Used to identify the specified QR code.
  - `frame2`: previous frame.
  - `contours`: Used to detect specified moving objects.
  - `circles`: Used to detect the specified blue ball.
  - `faces`: for detecting faces.
  - `gray`: for detecting faces.
  - `status`: for detecting falls.

### if_has_fall

- **Prototype**: `if_has_fall(status)`

- **Description**: Detect if a person has fallen.

- **Parameters**

  - `status` : Return value from `process_frame` function.

- **Return**
  - `{"type":"fall_people","data":True}`: Falling person detected
  - `None`: no one fell.

### if_has_face

- **Prototype**: `if_has_face(faces, gray, frame)`

- **Description**: Detect faces.

- **Parameters**

  - `faces` : Return value from `process_frame` function.
  - `gray`: Return value from `process_frame` function.
  - `frame` : Video frame.

- **Return**
  - `{"type":"human","data":face_data}`: face detected.
  - `None` : face not detected.

### if_has_ball

- **Prototype**: `if_has_ball(circles)`

- **Description**: Detect the specified blue ball.

- **Parameters**

  - `circles` : Return value from `process_frame` function.

- **Return**
  - `{"type":"ball","data":data}`: blue ball detected.
  - `None` : blue ball not detected.

### if_has_moving_obj

- **Prototype**: `if_has_moving_obj(contours)`

- **Description**: Detect specified toys.

- **Parameters**

  - `contours` : Return value from `process_frame` function.

- **Return**
  - `{"type":"obj","data":data}`: The specified toy was detected.
  - `None` : The specified toy was not detected.

### if_has_qrcode

- **Prototype**: `if_has_qrcode(corners, ids)`

- **Description**: Detect specified toys.

- **Parameters**

  - `corners` : Return value from `process_frame` function.
  - `ids` : Return value from `process_frame` function.

- **Return**
  - `{"type":"qrcode","data":data}`: The specified QR code was detected.
  - `None` : The specified QR code was not detected.

### detect_fall_people

- **Prototype**: `detect_fall_people(frame)`

- **Description**: Detect people falling.

- **Parameters**

  - `frame` : video frame.

- **Return**
  - `True`: A fall is detected.
  - `False` : No fall detected.

### get_face_ball_obj_qrcode

- **Prototype**: `get_face_ball_obj_qrcode()`

- **Description**: Integrated visual detection functions, including face, QR code, toys, fall detection.

- **Return**
  - `dictionary data`: detected.
  - `None` : Nothing detected.

## Instructions

### Integrated detection

```python
import vision.vision_base

vs = vision.vision_base.Vision()
while True:
    res = vs.get_face_ball_obj_qrcode()
```

### Detect faces

```python
import cv2
import vision.vision_base

vs = vision.vision_base.Vision()
cap = cv2.VideoCapture(0)
cap.set(3, vs.cam_width)
cap.set(4, vs.cam_height)

while True:
    frame = cap.read()[1]
    corners, ids, frame2, contours, circles, faces, gray, status = vs.process_frame(frame)
    result = vs.if_has_face(faces, gray, frame)
    print(result)
    k = cv2.waitKey(1) & 0xff
    if k == 27:
        cap.release()
        break
```
