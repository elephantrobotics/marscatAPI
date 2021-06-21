import spidev
import RPi.GPIO as GPIO
import numpy as np
import time


#SSD1351
SSD1351_WIDTH               = 128 #128
SSD1351_HEIGHT              = 128 #128
SSD1351_CMD_SETCOLUMN       = 0x15
SSD1351_CMD_SETROW          = 0x75
SSD1351_CMD_WRITERAM        = 0x5C
SSD1351_CMD_READRAM         = 0x5D
SSD1351_CMD_SETREMAP        = 0xA0
SSD1351_CMD_STARTLINE       = 0xA1
SSD1351_CMD_DISPLAYOFFSET   = 0xA2
SSD1351_CMD_DISPLAYALLOFF   = 0xA4
SSD1351_CMD_DISPLAYALLON    = 0xA5
SSD1351_CMD_NORMALDISPLAY   = 0xA6
SSD1351_CMD_INVERTDISPLAY   = 0xA7
SSD1351_CMD_FUNCTIONSELECT  = 0xAB
SSD1351_CMD_DISPLAYOFF      = 0xAE
SSD1351_CMD_DISPLAYON       = 0xAF
SSD1351_CMD_PRECHARGE       = 0xB1
SSD1351_CMD_DISPLAYENHANCE  = 0xB2
SSD1351_CMD_CLOCKDIV        = 0xB3
SSD1351_CMD_SETVSL          = 0xB4
SSD1351_CMD_SETGPIO         = 0xB5
SSD1351_CMD_PRECHARGE2      = 0xB6
SSD1351_CMD_SETGRAY         = 0xF8#灰度
SSD1351_CMD_USELUT          = 0xB9
SSD1351_CMD_PRECHARGELEVEL  = 0xBB
SSD1351_CMD_VCOMH           = 0xBE
SSD1351_CMD_CONTRASTABC     = 0xC1
SSD1351_CMD_CONTRASTMASTER  = 0xC7
SSD1351_CMD_MUXRATIO        = 0xCA
SSD1351_CMD_COMMANDLOCK     = 0xFD
SSD1351_CMD_HORIZSCROLL     = 0x96
SSD1351_CMD_STOPSCROLL      = 0x9E
SSD1351_CMD_STARTSCROLL     = 0x9F


#color
BLACK   = 0x0000
BLUE    = 0x001F
RED     = 0xF800
GREEN   = 0x07E0
CYAN    = 0x07FF
MAGENTA = 0xF81F
YELLOW  = 0xFFE0
WHITE   = 0xFFFF
#buffer
color_byte = [0x00, 0x00]
color_fill_byte = [0x00, 0x00]*(SSD1351_WIDTH)


OLED_RST_PIN = 27
OLED_DC_PIN  = 25
OLED_CS_PIN  = 8
#SPI init
SPI = spidev.SpiDev(0, 0)
SPI.max_speed_hz = 22000000 #40000000
SPI.mode = 0b00


def Set_Color(color):
    color_byte[0] = (color >> 8) & 0xff
    color_byte[1] = color & 0xff


def OLED_RST(x):
    GPIO.output(OLED_RST_PIN, x)


def OLED_DC(x):
    GPIO.output(OLED_DC_PIN, x)


def OLED_CS(x):
    GPIO.output(OLED_CS_PIN, x)


def Write_Command(cmd):
    OLED_CS(0)
    OLED_DC(0)
    SPI.writebytes([cmd])
    OLED_CS(1)


def Write_Data(data):
    OLED_CS(0)
    OLED_DC(1)
    SPI.writebytes([data])
    OLED_CS(1)


def Write_Datas(data):
    OLED_CS(0)
    OLED_DC(1)
    SPI.writebytes(data)
    OLED_CS(1)


def Write_Datas_Fast(spi_bytes):
    OLED_CS(0)
    OLED_DC(1)
    SPI.writebytes2(spi_bytes)
    OLED_CS(1)


def RAM_Address():
    Write_Command(0x15)
    Write_Data(0x00)
    Write_Data(0x7f)
    Write_Command(0x75)
    Write_Data(0x00)
    Write_Data(0x7f)


def Clear_Screen():
    RAM_Address()
    Write_Command(0x5c)
    OLED_CS(0)
    OLED_DC(1)
    color_fill_byte = [0x00, 0x00]*SSD1351_WIDTH*SSD1351_HEIGHT
    SPI.writebytes2(color_fill_byte)
    OLED_CS(1)


def Set_Coordinate(x, y):
    if((x >= SSD1351_WIDTH) or (y >= SSD1351_HEIGHT)):
        return
    # Set x and y coordinate
    Write_Command(SSD1351_CMD_SETCOLUMN)
    Write_Data(x)
    Write_Data(SSD1351_WIDTH-1)
    Write_Command(SSD1351_CMD_SETROW)
    Write_Data(y)
    Write_Data(SSD1351_HEIGHT-1)
    Write_Command(SSD1351_CMD_WRITERAM)


def Set_Address(column, row):
    Write_Command(SSD1351_CMD_SETCOLUMN)
    Write_Data(column)  #X start
    Write_Data(column)  #X end
    Write_Command(SSD1351_CMD_SETROW)
    Write_Data(row)     #Y start
    Write_Data(row+7)   #Y end
    Write_Command(SSD1351_CMD_WRITERAM)


def Device_Init():
    #GPIO Set
    GPIO.setmode(GPIO.BCM)
    #GPIO init
    GPIO.setwarnings(False)
    GPIO.setup(OLED_RST_PIN, GPIO.OUT)
    GPIO.setup(OLED_DC_PIN, GPIO.OUT)
    GPIO.setup(OLED_CS_PIN, GPIO.OUT)

    OLED_CS(0)
    OLED_RST(0)
    time.sleep(0.500)
    OLED_RST(1)
    time.sleep(0.500)

    Write_Command(0xfd) # command lock
    Write_Data(0x12)
    Write_Command(0xfd) # command lock
    Write_Data(0xB1)

    Write_Command(0xae) # display off
    Write_Command(0xa4) # Normal Display mode

    Write_Command(0x15) # set column address
    Write_Data(0x00)    # column address start 00
    Write_Data(0x7f)    # column address end 95
    Write_Command(0x75) # set row address
    Write_Data(0x00)    # row address start 00
    Write_Data(0x7f)    # row address end 63

    Write_Command(0xB3)
    Write_Data(0xFF)

    Write_Command(0xCA)
    Write_Data(0x7F)

    Write_Command(0xa0) # set re-map & data format
    Write_Data(0x74)    # Horizontal address increment

    Write_Command(0xa1) # set display start line
    Write_Data(0x00)    # start 00 line

    Write_Command(0xa2) # set display offset
    Write_Data(0x00)

    Write_Command(0xAB)
    Write_Command(0x01)

    Write_Command(0xB4)
    Write_Data(0xA0)
    Write_Data(0xB5)
    Write_Data(0x55)

    Write_Command(0xC1)
    Write_Data(0xC8)
    Write_Data(0x80)
    Write_Data(0xC0)

    Write_Command(0xC7)
    Write_Data(0x0F)

    Write_Command(0xB1)
    Write_Data(0x32)

    Write_Command(0xB2)
    Write_Data(0xA4)
    Write_Data(0x00)
    Write_Data(0x00)

    Write_Command(0xBB)
    Write_Data(0x17)

    Write_Command(0xB6)
    Write_Data(0x01)

    Write_Command(0xBE)
    Write_Data(0x05)

    Write_Command(0xA6)

    Clear_Screen()
    Write_Command(0xaf)


def display_image(image):
    Set_Coordinate(0, 0)
    b = np.asarray(image, dtype='uint8')
    a = np.empty((b.shape[0], b.shape[1], 2), dtype='uint8')
    a[:,:,[0]] = (b[:,:,[0]] & 0xF8) | (b[:,:,[1]] >> 5)
    a[:,:,[1]] = ((b[:,:,[1]] << 3) & 0xE0) | (b[:,:,[2]] >> 3)
    Write_Datas_Fast(a)