<!--
Copyright (c) 2019 Elephant Robotics, Inc. All rights reserved.

Using this MarsAI source code is subject to the terms and conditions of Apache 2.0 License. Check LICENSE for more information
-->

# Pyfirmata

Pyfirmata is the API for controlling micro-controller of all 16 servos and gyro sensor. You can use it to commnicate with each servo motor and control MarsCat move, run and turn.

## API


# Base Mode
SET_MODE					(0X10)	// data_1: mode (0: start_up; 1: send angles; 2: trot; 3 : trun; 4 : crawl)

GET_MODE					(0X11)  // data_1: mode default zero || rdata_1: mode (0: start_up; 1: send angles; 2: trot; 3 : trun; 4 : crawl)

GET_GYRO					(0X12)	// data_1: 0/1-RX/RY ; 3/4/5-AX/AY/AZ; 

GET_TOF						(0X13)	// rdata_1: tof_data

GET_BATTERY					(0X14)	// rdata_1: 0 battery percentage;  1 battery voltage

GET_SERVO_DATA				(0x15)  // data_1: servo_no (0 - all servo; 1~16 servo) ;data_2: servo_data: 0- ping; 1- temperature; 2-move; 3-current; 4-load; other:byte_id

SET_SERVO_DATA				(0x16)  // data_1: servo_no (0-all); data_2: byte_id, data_3: byte_data)

ENABLE_SERVOS				(0X17)	// data_1: servo_no (0 - all servo; 1~16 servo) ; data_2: servo_state(0 - power off;  1 - power on; 2- foucs)

# Basic Control
SET_HEAD_ANGLE				(0X20)	// data_1: joint_no; data_2: angle; data_3:speed

GET_HEAD_ANGLE				(0X21)	// rdata_1: joint_angle; 

SET_TAIL_ANGLE				(0X22)	// data_1: joint_no;data_2: angle; data_3:speed

GET_TAIL_ANGLE				(0X23)	// rdata_1: joint_angle

SET_LEG_ANGLE				(0X24)	// data_1: leg_no; data_2: joint_no; data_3: angle; data_4: speed

GET_LEG_ANGLE				(0X25)	// rdata_1: leg_no; rdata_2: joint_angle;

SET_LEG_OFFSET				(0X2A)	// data_1: leg_no; data_2: joint_no; data_3 dx, data_4 dy ;data_5 speed; data_6: mode(swing_or_trans);

SET_COG_OFFSET				(0X2B)	// data_1 dx; data_2 dy ; data_3: dz; data_4 speed; 

# Basic Move
SET_TROT					(0X30)	// data_1: step_no; data_2: dx ; data_3: dy; data_4: theta; data_5: speed; data_6: times (0 unlimited-max30);  

SET_TRUN					(0X31)	// data_1: rotation_angle; data_2: speed

SET_CRAWL					(0X32)	// data_1: step_no; data_2: dx ; data_3: dy; data_4: theta; data_5: speed; data_6: times (0 unlimited-max30);  

SET_TIME_DELAY				(0X3A)	// data_1: delay_time_base  -- only be byte

GET_TIME_DELAY				(0X3B)	// 

SET_STOP					(0X3F)  // stop data immediately 

# Calibration 
SET_CALIBRATION				(0X40)	// set calibration

GET_CALIBRATION				(0X41)	// get calibration

# Test
TEST_BODY_MOVE				(0X50)	// data_1: (0 - leg; 1 - haed; 2 - tail; 3 - body; 11 - special 1)


## Examples

Check example file to test the commnucation between python to micro-controller.

- important: the MarsCat is needed to test.

