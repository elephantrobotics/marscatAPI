from __future__ import division, unicode_literals

import inspect
import time

import serial

import struct

from .util import pin_list_to_board_dict, to_two_bytes, two_byte_iter_to_str

# Message command bytes (0x80(128) to 0xFF(255)) - straight from Firmata.h
DIGITAL_MESSAGE = 0x90      # send data for a digital pin
ANALOG_MESSAGE = 0xE0       # send data for an analog pin (or PWM)
DIGITAL_PULSE = 0x91        # SysEx command to send a digital pulse

# PULSE_MESSAGE = 0xA0      # proposed pulseIn/Out msg (SysEx)
# SHIFTOUT_MESSAGE = 0xB0   # proposed shiftOut msg (SysEx)
REPORT_ANALOG = 0xC0        # enable analog input by pin #
REPORT_DIGITAL = 0xD0       # enable digital input by port pair
START_SYSEX = 0xF0          # start a MIDI SysEx msg
SET_PIN_MODE = 0xF4         # set a pin to INPUT/OUTPUT/PWM/etc
END_SYSEX = 0xF7            # end a MIDI SysEx msg
REPORT_VERSION = 0xF9       # report firmware version
SYSTEM_RESET = 0xFF         # reset from MIDI
QUERY_FIRMWARE = 0x79       # query the firmware name

# extended command set using sysex (0-127/0x00-0x7F)
# 0x00-0x0F reserved for user-defined commands */

EXTENDED_ANALOG = 0x6F          # analog write (PWM, Servo, etc) to any pin
PIN_STATE_QUERY = 0x6D          # ask for a pin's current mode and value
PIN_STATE_RESPONSE = 0x6E       # reply with pin's current mode and value
CAPABILITY_QUERY = 0x6B         # ask for supported modes and resolution of all pins
CAPABILITY_RESPONSE = 0x6C      # reply with supported modes and resolution
ANALOG_MAPPING_QUERY = 0x69     # ask for mapping of analog to pin numbers
ANALOG_MAPPING_RESPONSE = 0x6A  # reply with mapping info

SERVO_CONFIG = 0x70         # set max angle, minPulse, maxPulse, freq
STRING_DATA = 0x71          # a string message with 14-bits per char
SHIFT_DATA = 0x75           # a bitstream to/from a shift register
I2C_REQUEST = 0x76          # send an I2C read/write request
I2C_REPLY = 0x77            # a reply to an I2C read request
I2C_CONFIG = 0x78           # config I2C settings such as delay times and power pins
REPORT_FIRMWARE = 0x79      # report name and version of the firmware
SAMPLING_INTERVAL = 0x7A    # set the poll rate of the main loop
SYSEX_NON_REALTIME = 0x7E   # MIDI Reserved for non-realtime messages
SYSEX_REALTIME = 0x7F       # MIDI Reserved for realtime messages

# Mars code 
MARS = 0XAA

# Base Mode
SET_MODE        = 0X10  
GET_MODE        = 0x11 
GET_GYRO        = 0X12  
GET_TOF         = 0X13 
GET_BATTERY     = 0X14 
GET_SERVO_DATA  = 0x15 
SET_SERVO_DATA	= 0x16 
ENABLE_SERVOS   = 0X17  

# Basic Control
SET_HEAD_ANGLE  = 0X20  
GET_HEAD_ANGLE  = 0X21  
SET_TAIL_ANGLE  = 0X22 
GET_TAIL_ANGLE  = 0X23  
SET_LEG_ANGLE   = 0X24 
GET_LEG_ANGLE   = 0X25
SET_LEG_OFFSET  = 0X2A  
SET_COG_OFFSET  = 0X2B 

# Basic Move
SET_TROT        = 0X30 
SET_TRUN        = 0X31 
SET_CRAWL       = 0X32  
SET_TIME_DELAY  = 0X3A 
GET_TIME_DELAY  = 0X3B  # do not use
SET_STOP        = 0X3F  # do not use

# Calibration 
SET_CALIBRATION = 0X40  
GET_CALIBRATION = 0X41  

# Test
TEST_BODY_MOVE  = 0X50  

serial_sync_time    = 0.1
error_no            = -1000
lstep_error         = 1 #lenght of step error number
default_dx          = 0.04
default_dy          = 0
# Mars end  


# Pin modes.
# except from UNAVAILABLE taken from Firmata.h
UNAVAILABLE = -1
INPUT = 0          # as defined in wiring.h
OUTPUT = 1         # as defined in wiring.h
ANALOG = 2         # analog pin in analogInput mode
PWM = 3            # digital pin in PWM output mode
SERVO = 4          # digital pin in SERVO mode

# Pin types
DIGITAL = OUTPUT   # same as OUTPUT below
# ANALOG is already defined above

# Time to wait after initializing serial, used in Board.__init__
BOARD_SETUP_WAIT_TIME = 5


class PinAlreadyTakenError(Exception):
    pass


class InvalidPinDefError(Exception):
    pass


class NoInputWarning(RuntimeWarning):
    pass


class Board(object):
    """The Base class for any board."""
    firmata_version = None
    firmware = None
    firmware_version = None
    _command_handlers = {}
    _command = None
    _stored_data = []
    _parsing_sysex = False

    def __init__(self, port, layout=None, baudrate=57600, name=None, timeout=0.5, write_timeout=0.5):
        self.sp = serial.Serial(port, baudrate, timeout=timeout, write_timeout=write_timeout)
        # Allow 5 secs for Arduino's auto-reset to happen
        # Alas, Firmata blinks its version before printing it to serial
        # For 2.3, even 5 seconds might not be enough.
        # TODO Find a more reliable way to wait until the board is ready
        self.pass_time(BOARD_SETUP_WAIT_TIME)
        self.name = name
        self._layout = layout
        if not self.name:
            self.name = port

        if layout:
            self.setup_layout(layout)
        else:
            self.auto_setup()

        # Iterate over the first messages to get firmware data
        while self.bytes_available():
            self.iterate()
        # TODO Test whether we got a firmware name and version, otherwise there
        # probably isn't any Firmata installed

    def __str__(self):
        return "Board{0.name} on {0.sp.port}".format(self)

    def __del__(self):
        """
        The connection with the a board can get messed up when a script is
        closed without calling board.exit() (which closes the serial
        connection). Therefore also do it here and hope it helps.
        """
        self.exit()

    def send_as_two_bytes(self, val):
        self.sp.write(bytearray([val % 128, val >> 7]))

    def setup_layout(self, board_layout):
        """
        Setup the Pin instances based on the given board layout.
        """
        # Create pin instances based on board layout
        self.analog = []
        for i in board_layout['analog']:
            self.analog.append(Pin(self, i))

        self.digital = []
        self.digital_ports = []
        for i in range(0, len(board_layout['digital']), 8):
            num_pins = len(board_layout['digital'][i:i + 8])
            port_number = int(i / 8)
            self.digital_ports.append(Port(self, port_number, num_pins))

        # Allow to access the Pin instances directly
        for port in self.digital_ports:
            self.digital += port.pins

        # Setup PWM pins
        for i in board_layout['pwm']:
            self.digital[i].PWM_CAPABLE = True

        # Disable certain ports like Rx/Tx and crystal ports
        for i in board_layout['disabled']:
            self.digital[i].mode = UNAVAILABLE

        # Create a dictionary of 'taken' pins. Used by the get_pin method
        self.taken = {'analog': dict(map(lambda p: (p.pin_number, False), self.analog)),
                      'digital': dict(map(lambda p: (p.pin_number, False), self.digital))}

        self._set_default_handlers()

    def _set_default_handlers(self):
        # Setup default handlers for standard incoming commands
        self.add_cmd_handler(ANALOG_MESSAGE, self._handle_analog_message)
        self.add_cmd_handler(DIGITAL_MESSAGE, self._handle_digital_message)
        self.add_cmd_handler(REPORT_VERSION, self._handle_report_version)
        self.add_cmd_handler(REPORT_FIRMWARE, self._handle_report_firmware)

    def auto_setup(self):
        """
        Automatic setup based on Firmata's "Capability Query"
        """
        self.add_cmd_handler(CAPABILITY_RESPONSE, self._handle_report_capability_response)
        self.send_sysex(CAPABILITY_QUERY, [])
        self.pass_time(0.1)  # Serial SYNC

        while self.bytes_available():
            self.iterate()

        # handle_report_capability_response will write self._layout
        if self._layout:
            self.setup_layout(self._layout)
        else:
            raise IOError("Board detection failed.")

    def add_cmd_handler(self, cmd, func):
        """Adds a command handler for a command."""
        len_args = len(inspect.getargspec(func)[0])

        def add_meta(f):
            def decorator(*args, **kwargs):
                f(*args, **kwargs)
            decorator.bytes_needed = len_args - 1  # exclude self
            decorator.__name__ = f.__name__
            return decorator
        func = add_meta(func)
        self._command_handlers[cmd] = func

    def get_pin(self, pin_def):
        """
        Returns the activated pin given by the pin definition.
        May raise an ``InvalidPinDefError`` or a ``PinAlreadyTakenError``.

        :arg pin_def: Pin definition as described below,
            but without the arduino name. So for example ``a:1:i``.

        'a' analog pin     Pin number   'i' for input
        'd' digital pin    Pin number   'o' for output
                                        'p' for pwm (Pulse-width modulation)

        All seperated by ``:``.
        """
        if type(pin_def) == list:
            bits = pin_def
        else:
            bits = pin_def.split(':')
        a_d = bits[0] == 'a' and 'analog' or 'digital'
        part = getattr(self, a_d)
        pin_nr = int(bits[1])
        if pin_nr >= len(part):
            raise InvalidPinDefError('Invalid pin definition: {0} at position 3 on {1}'
                                     .format(pin_def, self.name))
        if getattr(part[pin_nr], 'mode', None) == UNAVAILABLE:
            raise InvalidPinDefError('Invalid pin definition: '
                                     'UNAVAILABLE pin {0} at position on {1}'
                                     .format(pin_def, self.name))
        if self.taken[a_d][pin_nr]:
            raise PinAlreadyTakenError('{0} pin {1} is already taken on {2}'
                                       .format(a_d, bits[1], self.name))
        # ok, should be available
        pin = part[pin_nr]
        self.taken[a_d][pin_nr] = True
        if pin.type is DIGITAL:
            if bits[2] == 'p':
                pin.mode = PWM
            elif bits[2] == 's':
                pin.mode = SERVO
            elif bits[2] != 'o':
                pin.mode = INPUT
        else:
            pin.enable_reporting()
        return pin

    def pass_time(self, t):
        """Non-blocking time-out for ``t`` seconds."""
        #time.sleep(t)
        cont = time.time() + t
        while time.time() < cont:
            time.sleep(0)
        

    def send_sysex(self, sysex_cmd, data):
        """
        Sends a SysEx msg.

        :arg sysex_cmd: A sysex command byte
        : arg data: a bytearray of 7-bit bytes of arbitrary data
        """
        msg = bytearray([START_SYSEX, sysex_cmd])
        msg.extend(data)
        msg.append(END_SYSEX)
        self.sp.write(msg)

    def bytes_available(self):
        return self.sp.inWaiting()

    def iterate(self, mode_id = 0x00, verfiy = 0):
        """
        Reads and handles data from the microcontroller over the serial port.
        This method should be called in a main loop or in an :class:`Iterator`
        instance to keep this boards pin values up to date.
        """
        try:
            byte = self.sp.read()
        except Exception as e:
            return

        if not byte:
            return

        #Get the first data
        data = ord(byte)

        received_data = []
        handler = None

        if data < START_SYSEX:
            # These commands can have 'channel data' like a pin nummber appended.
            try:
                handler = self._command_handlers[data & 0xF0]
            except KeyError:
                return
            received_data.append(data & 0x0F)
            while len(received_data) < handler.bytes_needed:
                try:
                    received_data.append(ord(self.sp.read()))
                except Exception as e:
                    return
        elif data == START_SYSEX:
            try:
                data = ord(self.sp.read())
            except Exception as e:
                return

            # Check the data rec position
            if data != mode_id: 
                return None

            #handler = self._command_handlers.get(data)
            #if not handler:
            #    return

            # Acquire data
            if data > ANALOG_MAPPING_RESPONSE or data < SET_MODE:
                return
            try:
                data = ord(self.sp.read())
            except Exception as e:
                return

            while data != END_SYSEX:
                received_data.append(data)
                try:
                    data = ord(self.sp.read())
                except Exception as e:
                    return
                if len(received_data) > 20:  #possible data len is max 20
                    return

            if ((mode_id == 0x21) or (mode_id == 0x23) or (mode_id == 0x25)):
                verfiy =  received_data[0]
                data_processed = self.processDataList(received_data[1:5])

                return (verfiy, data_processed)
            else:
                data_processed = self.processDataList(received_data)
                return (data_processed)
            # End Acquire

        else:
            try:
                handler = self._command_handlers[data]
            except KeyError:
                return
            while len(received_data) < handler.bytes_needed:
                try:
                    received_data.append(ord(self.sp.read()))
                except Exception as e:
                    return

        # Handle the data
        '''
        try:
            handler(*received_data)
        except ValueError:
            pass
        '''


    def get_firmata_version(self):
        """
        Returns a version tuple (major, minor) for the firmata firmware on the
        board.
        """
        return self.firmata_version

    def servo_config(self, pin, min_pulse=544, max_pulse=2400, angle=0):
        """
        Configure a pin as servo with min_pulse, max_pulse and first angle.
        ``min_pulse`` and ``max_pulse`` default to the arduino defaults.
        """
        if pin > len(self.digital) or self.digital[pin].mode == UNAVAILABLE:
            raise IOError("Pin {0} is not a valid servo pin".format(pin))

        data = bytearray([pin])
        data += to_two_bytes(min_pulse)
        data += to_two_bytes(max_pulse)
        self.send_sysex(SERVO_CONFIG, data)

        # set pin._mode to SERVO so that it sends analog messages
        # don't set pin.mode as that calls this method
        self.digital[pin]._mode = SERVO
        self.digital[pin].write(angle)

    def exit(self):
        """Call this to exit cleanly."""
        # First detach all servo's, otherwise it somehow doesn't want to close...
        if hasattr(self, 'digital'):
            for pin in self.digital:
                if pin.mode == SERVO:
                    pin.mode = OUTPUT
        if hasattr(self, 'sp'):
            self.sp.close()

    # Command handlers
    def _handle_analog_message(self, pin_nr, lsb, msb):
        value = round(float((msb << 7) + lsb) / 1023, 4)
        # Only set the value if we are actually reporting
        try:
            if self.analog[pin_nr].reporting:
                self.analog[pin_nr].value = value
        except IndexError:
            raise ValueError

    def _handle_digital_message(self, port_nr, lsb, msb):
        """
        Digital messages always go by the whole port. This means we have a
        bitmask which we update the port.
        """
        mask = (msb << 7) + lsb
        try:
            self.digital_ports[port_nr]._update(mask)
        except IndexError:
            raise ValueError

    def _handle_report_version(self, major, minor):
        self.firmata_version = (major, minor)

    def _handle_report_firmware(self, *data):
        major = data[0]
        minor = data[1]
        self.firmware_version = (major, minor)
        self.firmware = two_byte_iter_to_str(data[2:])

    def _handle_report_capability_response(self, *data):
        charbuffer = []
        pin_spec_list = []

        for c in data:
            if c == CAPABILITY_RESPONSE:
                continue

            charbuffer.append(c)
            if c == 0x7F:
                # A copy of charbuffer
                pin_spec_list.append(charbuffer[:])
                charbuffer = []

        self._layout = pin_list_to_board_dict(pin_spec_list)



    '''

    --- Mars Library ---

    Communication function with MarsCat MicroController

    '''

    # Base Mode
    def setMode(self, mode = 0, speed = 1):  # 0x10
        '''
        mode: 
            0: start_up
            1: send angles
            2: trot 
            3: trun 
            4: crawl
            11: enable collision avoidance during moving
            12: disable collision avoidance during moving
        '''
        data = [SET_MODE]
        data += [mode]   
        data += self.getSpeedBytesData(speed)
        self.send_sysex(MARS,data)

    def getMode(self, mode_state = 1):  # 0x11
        '''
        mode_state: 
            0: real mode 
            1: version
            9: get ax_sqr_sum during one move
            
        rdata for getMode(0) - real modeo
            0: start_up 
            1: send angles
            2: trot
            3 : trun
            4 : crawl 
            8 : flip (abs(rx or ry) > 10)
            9 : hit wall/ find edge
        '''
        data = [GET_MODE]	# 1 means version
        data += [mode_state]    

        self.send_sysex(MARS, data)
        self.pass_time(serial_sync_time)  # Serial SYNC
        
        while self.bytes_available():
            rdata = self.iterate(GET_MODE)
            if rdata != None:
                return "{:.2f}".format(rdata)
        return error_no - GET_MODE    # error number

    def getGyro(self, data_no):   # 0x12
        '''
        data_no:
             0/1-RX/RY 
             3/4/5-AX/AY/AZ

        rdata:
            gyro angle/ acceraltion 
        '''
        data = [GET_GYRO]
        data += [data_no]
        self.send_sysex(MARS, data)
	
        self.pass_time(0.24)  # Serial SYNC  -- made by Joey, let him know if changes
        
        while self.bytes_available():
            rdata = self.iterate(GET_GYRO)
            if rdata != None:
                return rdata
        return error_no - GET_GYRO     # error number


    def getTof(self):   # 0x13
        '''
        rdata_1: 
            tof_data
        '''
        data = [GET_TOF]
        self.send_sysex(MARS, data)
        self.pass_time(0.07)  # Serial SYNC change to 0.07 to speed up

        while self.bytes_available():
            rdata = self.iterate(GET_TOF)
            if rdata != None:
                return rdata
        
        return error_no - GET_TOF    # error number

    def getBattery(self, battery_or_perc = 1):   # 0x14
        '''
        battery_or_perc:
            0 battery percentage
            1 battery voltage

        rdata_1: 
            battery data
        '''
        data = [GET_BATTERY]
        data += [battery_or_perc]
        
        self.send_sysex(MARS, data)
        self.pass_time(serial_sync_time)  # Serial SYNC

        while self.bytes_available():
            rdata = self.iterate(GET_BATTERY)
            if rdata != None:
                return rdata
        
        return error_no - GET_BATTERY

    def getServoData(self, servo_id, data_type):   # 0x15
        '''
        servo_id: 
            0 - all servo 
            1~16 servo
        data_type:  
            0- ping
            1- temperature
            2-move
            3-current
            4-load
            other-byte_id

        rdata:
            servo_data
        '''
        data = [GET_SERVO_DATA]
        data += [servo_id]
        data += [data_type]
        self.send_sysex(MARS, data)
        self.pass_time(serial_sync_time)  # Serial SYNC

        while self.bytes_available():       
            rdata = self.iterate(GET_SERVO_DATA)      
            if rdata != None:
                return rdata
        
        return error_no - GET_SERVO_DATA

    def setServoData(self, servo_id, data_id, l_data):
        '''
        servo_id: 
            servo_no (0-all)
        data_id: 
            byte_id 
        l_data: 
            byte_data (max 128)
        '''

        data = [SET_SERVO_DATA]
        data += [servo_id]
        data += [data_id]
        data += [l_data]
        self.send_sysex(MARS, data)
	

    def enableServos(self,servo_no, servo_state):   # 0x17
        '''
        servo_no
            0 - all servo
            1~16 servo 
        servo_state
            0 - power off  
            1 - power on 
            2 - foucs
        '''
        data = [ENABLE_SERVOS]
        data += [servo_no]
        data += [servo_state]

        self.send_sysex(MARS, data)


    # Basic Control
    def setHeadAngle(self, joint_no, angle, speed=1): # 0x20
        '''
        joint_no
            1 or 2
        angle
            -180~180
        speed
            0~1
        '''
        data = [SET_HEAD_ANGLE]
        data += [joint_no]
        data += struct.pack("f", angle-10)
        data += self.getSpeedBytesData(speed)
        self.send_sysex(MARS,data)

    def getHeadAngle(self, joint_no):  # 0x21
        '''
        joint_no
            1 or 2

        rdata 
            joint_angle    
        '''
        data = [GET_HEAD_ANGLE]
        data += [joint_no]
        
        self.send_sysex(MARS, data)
        self.pass_time(serial_sync_time)  # Serial SYNC

        while self.bytes_available():
            rdata = self.iterate(GET_HEAD_ANGLE)
            if rdata != None:
                if rdata[0] == joint_no:
                    if rdata[1]!= None:
                        return  rdata[1]
        
        return error_no - GET_HEAD_ANGLE

    def setTailAngle(self, joint_no, angle, speed):   # 0x22
        '''
        joint_no
            1 or 2
        angle
            -180 ~ 180
        speed
            0 ~ 1
        '''
        data = [SET_TAIL_ANGLE]
        data += [joint_no]
        data += struct.pack("f", angle)
        data += self.getSpeedBytesData(speed)
        self.send_sysex(MARS,data)
        

    def getTailAngle(self, joint_no):  # 0x23
        '''
        joint_no
            1 or 2

        rdata 
            joint_angle  
        '''
        data = [GET_TAIL_ANGLE]
        data += [joint_no]
        
        self.send_sysex(MARS, data)
        self.pass_time(serial_sync_time)  # Serial SYNC

        while self.bytes_available():
            rdata = self.iterate(GET_TAIL_ANGLE)
            if rdata != None:
                if rdata[0] == joint_no:
                    if rdata[1] != None:
                        return  rdata[1]
                
        
        return error_no - GET_TAIL_ANGLE

    def setLegAngle(self, leg_no, joint_no, angle, speed):   # 0x24
        '''
        leg_no
            1 - 4
        joint_no
            1  -3
        angle
            -180 - 180
        speed
            0 - 1
        '''
        data = [SET_LEG_ANGLE]
        data += [leg_no]
        data += [joint_no]
        data += struct.pack("f", angle)
        data += self.getSpeedBytesData(speed)
        self.send_sysex(MARS,data)
        

    def getLegAngle(self, leg_no, joint_no):   # 0x25
        '''
        leg_no
            1 - 4
        joint_no
            1 -3

        rdata_1: 
            joint_angle
        '''
        data = [GET_LEG_ANGLE]
        data += [leg_no]
        data += [joint_no]
        self.send_sysex(MARS, data)
        self.pass_time(serial_sync_time)  # Serial SYNC
        
        while self.bytes_available():
            rdata = self.iterate(GET_LEG_ANGLE)
            if rdata != None:
                servo_no = (leg_no -1)*4 + joint_no
                if rdata[0] == servo_no:
                    if rdata[1]!= None:
                        return  rdata[1]
        return error_no - GET_LEG_ANGLE


    def setLegOffset(self, leg_no, dx, dy, dz, speed):   # 0x2A
        '''
        leg_no
            1 - 4
        joint_no
            1 -3
        dx(safe range)
            -0.1 ~ 0.1
        dy (safe range)
            -0.05 ~ 0.0.5
        dz(safe range)
            -0.1 ~ 0.1
        speed
            0~1
        '''
        data = [SET_LEG_OFFSET]
        data += [leg_no]
        data += struct.pack("f", dx)
        data += struct.pack("f", dy)
        data += struct.pack("f", dz)
        data += self.getSpeedBytesData(speed)
        self.send_sysex(MARS,data)

    def setCOGOffset(self, dx, dy, dz, speed): # 0x2B
        '''
        dx(safe range)
            -0.1 ~ 0.1
        dy (safe range)
            -0.05 ~ 0.0.5
        dz(safe range)
            -0.1 ~ 0.1
        speed
            0~1
        '''
        data = [SET_COG_OFFSET]
        data += struct.pack("f", dx)
        data += struct.pack("f", dy)
        data += struct.pack("f", dz)
        data += self.getSpeedBytesData(speed)
        self.send_sysex(MARS, data)
             

    # Basic Move
    def setTrot(self, step_num = 0 , dx = default_dx, dy = default_dy, speed = 1.0, dtheta = 0.0, steps = 1): # 0x30
        '''
        step_no
             disable step no , don't need to input
        dx(safe range)
            -0.05 ~ 0.05
        dy(safe range)
            -0.05 ~ 0.05
        dtheta(safe range)
            -10 ~ 10
        speed
            0~1
        steps 
            1 ~ 30
        '''
        data = [SET_TROT]
        data += [step_num]
        data += struct.pack("f", dx)
        data += struct.pack("f", dy)
        data += struct.pack("f", dtheta)
        data += self.getSpeedBytesData(speed)
        data += [steps]
        self.send_sysex(MARS, data)

    def setTurn(self, rotate_angle, speed=1.0, steps = 1): # 0x31
        '''
        rotate_angle(safe range)
            -10 ~ 10
        speed
            0~1
        steps 
            1 ~ 30
        '''
        data = [SET_TRUN]
        data += struct.pack("f", rotate_angle)
        data += self.getSpeedBytesData(speed)
        data += struct.pack("f", steps)

        self.send_sysex(MARS, data)

    def setCrawl(self, step_num = 0, speed=1.0, dx = default_dx , dy = default_dx, dtheta = 0, steps = 1): # 0x32 # yapf: disable
        '''
        step_no
            disable step no , don't need to input
        dx(safe range)
            -0.04 ~ 0.04
        dy(safe range)
            -0.04 ~ 0.04
        dtheta(safe range)
            -5 ~ 5
        speed
            0~1
        steps 
            1 ~ 30
        '''
        data = [SET_CRAWL]
        data += [step_num]
        data += struct.pack("f", dx)
        data += struct.pack("f", dy)
        data += struct.pack("f", dtheta)
        data += self.getSpeedBytesData(speed)
        data += [steps]
        self.send_sysex(MARS, data)

    def setTimeDelay(self, time_msec): # 0x3A
        '''
        do not use
        '''
        data = [SET_TIME_DELAY]
        data += struct.pack("f", time_msec)
        
        self.send_sysex(MARS, data)
        

    def getTimeDelay(self): # 0x3B
        '''
        do not use
        '''
        data = [GET_TIME_DELAY]
        self.send_sysex(MARS, data)
        self.pass_time(serial_sync_time)  # Serial SYNC
        
        while self.bytes_available():
            rdata = self.iterate(GET_TIME_DELAY)
            if rdata != None:
                return rdata
        return error_no - GET_TIME_DELAY
        

    def setStop(self):  #0x3F
        '''
        do not use
        '''
        data = [SET_STOP]
        self.send_sysex(MARS, data)
        

    # Calibration
    def setCalibration(self): # 0x40
        '''
        set calibration
        '''
        data = [SET_CALIBRATION]
        self.send_sysex(MARS, data)
        

    def getCalibration(self): # 0x41
        '''
        get calibration

        rdata_1
            0 : not calibrated 
            1: calibrated

        '''
        data = [GET_CALIBRATION]
        self.send_sysex(MARS, data)
        
        while self.bytes_available():
            rdata = self.iterate(GET_CALIBRATION)
            if rdata != None:
                return rdata
        return error_no - GET_CALIBRATION


    # Test
    def testBodyMove(self, mode): # 0x50
        '''
        mode:
            0 - leg
            1 - haed
            2 - tail
            3 - body
            11 - special
        '''
        data = [TEST_BODY_MOVE]
        data += [mode]
        self.send_sysex(MARS, data)


    def processDataList(self,received_data):

        if len(received_data) == 4:
            data_bt = bytearray(received_data)
            stu = struct.unpack("<f", data_bt)
            return stu[0]

    def getSpeedBytesData(self, speed):
        bt_sp = int(speed * 100)
        if speed > 1:
            bt_sp = 100
        elif speed < 0:
            bt_sp = 0
        return [bt_sp]  



class Port(object):
    """An 8-bit port on the board."""
    def __init__(self, board, port_number, num_pins=8):
        self.board = board
        self.port_number = port_number
        self.reporting = False

        self.pins = []
        for i in range(num_pins):
            pin_nr = i + self.port_number * 8
            self.pins.append(Pin(self.board, pin_nr, type=DIGITAL, port=self))

    def __str__(self):
        return "Digital Port {0.port_number} on {0.board}".format(self)

    def enable_reporting(self):
        """Enable reporting of values for the whole port."""
        self.reporting = True
        msg = bytearray([REPORT_DIGITAL + self.port_number, 1])
        self.board.sp.write(msg)

        for pin in self.pins:
            if pin.mode == INPUT:
                pin.reporting = True  # TODO Shouldn't this happen at the pin?

    def disable_reporting(self):
        """Disable the reporting of the port."""
        self.reporting = False
        msg = bytearray([REPORT_DIGITAL + self.port_number, 0])
        self.board.sp.write(msg)

    def write(self):
        """Set the output pins of the port to the correct state."""
        mask = 0
        for pin in self.pins:
            if pin.mode == OUTPUT:
                if pin.value == 1:
                    pin_nr = pin.pin_number - self.port_number * 8
                    mask |= 1 << int(pin_nr)
#        print("type mask", type(mask))
#        print("type self.portnumber", type(self.port_number))
#        print("type pinnr", type(pin_nr))
        msg = bytearray([DIGITAL_MESSAGE + self.port_number, mask % 128, mask >> 7])
        self.board.sp.write(msg)

    def _update(self, mask):
        """Update the values for the pins marked as input with the mask."""
        if self.reporting:
            for pin in self.pins:
                if pin.mode is INPUT:
                    pin_nr = pin.pin_number - self.port_number * 8
                    pin.value = (mask & (1 << pin_nr)) > 0


class Pin(object):
    """A Pin representation"""
    def __init__(self, board, pin_number, type=ANALOG, port=None):
        self.board = board
        self.pin_number = pin_number
        self.type = type
        self.port = port
        self.PWM_CAPABLE = False
        self._mode = (type == DIGITAL and OUTPUT or INPUT)
        self.reporting = False
        self.value = None

    def __str__(self):
        type = {ANALOG: 'Analog', DIGITAL: 'Digital'}[self.type]
        return "{0} pin {1}".format(type, self.pin_number)

    def _set_mode(self, mode):
        if mode is UNAVAILABLE:
            self._mode = UNAVAILABLE
            return
        if self._mode is UNAVAILABLE:
            raise IOError("{0} can not be used through Firmata".format(self))
        if mode is PWM and not self.PWM_CAPABLE:
            raise IOError("{0} does not have PWM capabilities".format(self))
        if mode == SERVO:
            if self.type != DIGITAL:
                raise IOError("Only digital pins can drive servos! {0} is not"
                              "digital".format(self))
            self._mode = SERVO
            self.board.servo_config(self.pin_number)
            return

        # Set mode with SET_PIN_MODE message
        self._mode = mode
        self.board.sp.write(bytearray([SET_PIN_MODE, self.pin_number, mode]))
        if mode == INPUT:
            self.enable_reporting()

    def _get_mode(self):
        return self._mode

    mode = property(_get_mode, _set_mode)
    """
    Mode of operation for the pin. Can be one of the pin modes: INPUT, OUTPUT,
    ANALOG, PWM. or SERVO (or UNAVAILABLE).
    """

    def enable_reporting(self):
        """Set an input pin to report values."""
        if self.mode is not INPUT:
            raise IOError("{0} is not an input and can therefore not report".format(self))
        if self.type == ANALOG:
            self.reporting = True
            msg = bytearray([REPORT_ANALOG + self.pin_number, 1])
            self.board.sp.write(msg)
        else:
            self.port.enable_reporting()
            # TODO This is not going to work for non-optimized boards like Mega

    def disable_reporting(self):
        """Disable the reporting of an input pin."""
        if self.type == ANALOG:
            self.reporting = False
            msg = bytearray([REPORT_ANALOG + self.pin_number, 0])
            self.board.sp.write(msg)
        else:
            self.port.disable_reporting()
            # TODO This is not going to work for non-optimized boards like Mega

    def read(self):
        """
        Returns the output value of the pin. This value is updated by the
        boards :meth:`Board.iterate` method. Value is always in the range from
        0.0 to 1.0.
        """
        if self.mode == UNAVAILABLE:
            raise IOError("Cannot read pin {0}".format(self.__str__()))
        return self.value

    def write(self, value):
        """
        Output a voltage from the pin

        :arg value: Uses value as a boolean if the pin is in output mode, or
            expects a float from 0 to 1 if the pin is in PWM mode. If the pin
            is in SERVO the value should be in degrees.

        """
        if self.mode is UNAVAILABLE:
            raise IOError("{0} can not be used through Firmata".format(self))
        if self.mode is INPUT:
            raise IOError("{0} is set up as an INPUT and can therefore not be written to"
                          .format(self))
        if value is not self.value:
            self.value = value
            if self.mode is OUTPUT:
                if self.port:
                    self.port.write()
                else:
                    msg = bytearray([DIGITAL_MESSAGE, self.pin_number, value])
                    self.board.sp.write(msg)
            elif self.mode is PWM:
                value = int(round(value * 255))
                msg = bytearray([ANALOG_MESSAGE + self.pin_number, value % 128, value >> 7])
                self.board.sp.write(msg)
            elif self.mode is SERVO:
                value = int(value)
                msg = bytearray([ANALOG_MESSAGE + self.pin_number, value % 128, value >> 7])
                self.board.sp.write(msg)
