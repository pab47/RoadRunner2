#!/usr/bin/python3
# Remote that sets an angle as a target and uses a PID library to stay at that angle. Based off
# work by FOA and Pranav Bhounsule

# Eric Sanchez 8-6-2018

# PWM
from __future__ import division, print_function
import time as time_
# ODrive
import odrive
from odrive.enums import *
import math
# Joystick
import os, struct, array
from fcntl import ioctl
import threading
# Shell commands
from subprocess import call
import sys
# PID
import numpy as np
# from scipy.interpolate import spline
# 9 Axis Abs Orientation Sensor
from Adafruit_BNO055 import BNO055
# Multiprocessing
from multiprocessing import Process, Pipe
# Serial Comms
import serial, select
# Reset Teensy
import wiringpi

# ----------------------------------------------------
# IMU Code -------------------------------------------

# Init. BNO055 object
bno = BNO055.BNO055(serial_port='/dev/serial0', rst=18)

# Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

# Print system status and self test result.
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
# Print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')

# Axis remap values
AXIS_REMAP_X                         = 0x00
AXIS_REMAP_Y                         = 0x01
AXIS_REMAP_Z                         = 0x02
AXIS_REMAP_POSITIVE                  = 0x00
AXIS_REMAP_NEGATIVE                  = 0x01

bno.set_axis_remap(AXIS_REMAP_Y,AXIS_REMAP_X,AXIS_REMAP_Z,AXIS_REMAP_POSITIVE,AXIS_REMAP_NEGATIVE,AXIS_REMAP_POSITIVE)

# Instatiate Variables
heading, roll, pitch = bno.read_euler()
pitchrate = 0.0
last_pitch = 0.0

# ----------------------------------------------------
# WiringPi Code --------------------------------------

wiringpi.wiringPiSetup() 
wiringpi.pinMode(4, 1)

# Reset Teensy
wiringpi.digitalWrite(4, 0)
time_.sleep(0.1)
wiringpi.digitalWrite(4, 1)
time_.sleep(0.75)

# ----------------------------------------------------
# Odrive Controller Code -----------------------------

# Find a connected ODrive (this will block until you connect one)
print("Finding an ODrive...")
my_drive = odrive.find_any()

print("Odrive found!")

calibrating = False

armed = False
latched = True

# ----- VELOCITY LIMIT -----
mph = 6 # Speed in MPH
vel_limit = mph*1.4666667/(3.1416*1.70833)*4.5*8192
print('Velocity limit is %i' % vel_limit)
# --------------------------

prev_speedL = 0
prev_speedR = 0
prev_speed = 0

m0pos = 0.0
m0pos_deg = 0.0
m0theta = 0.0
m0speed = 0.0
m0beta = 0.0

m1pos = 0.0
m1pos_deg = 0.0
m1theta = 0.0
m1speed = 0.0
m1beta = 0.0

# Encoder offsets for absolute angle

# Find the offset of the encoders (zeros out encoders) properly scaled with
# pulley ratio
m0pos_offset = (my_drive.axis0.encoder.pos_estimate)/4.5
m1pos_offset = (my_drive.axis1.encoder.pos_estimate)/4.5

# Find the offset for the absolute position of the wheel
# With two feet on the ground, the absolute position of the marked wheel should
# be 18 degrees (half of the angle between spokes)
beta0 = 18 - pitch

# ----------------------------------------------------
# PID Code -------------------------------------------

AUTOMATIC  = 1
MANUAL = 0
DIRECT = 0  # Default
REVERSE = 1
P_ON_M = 0
P_ON_E = 1  # Default

# kp = 2
# ki = 4*kp                 # Supposed to be a gen rule
# # ki = 3
# kd = 1

# PID Tuning

# BEST SO FAR
# kp = .1
# ki = 2
# # ki = 0
# # kd = 0.2543
# # kd=0
# kd= .1*kp

kp = .4
ki = 2.75
kd= .1*kp
# kd = 0

feedback = 0                # Initialize var
setpt_on = False
angle_cnt = 0
prev_output = 0
mapped_out = 0.0

curr_time = 0.0
prev_time = 0.0
curr_time_ard = 0.0
prev_time_ard = 0.0

# ----------------------------------------------------
# Serial Comm Code -----------------------------------

print('Opening Arduino serial comm...')
serPort = '/dev/ttyACM0'
# Serial (write)
ardSer = serial.Serial(serPort, 115200, timeout=0.1)
# USB CDC (read)
teenSer = open(serPort,'r')

# Initialize a poll object to check status of USB
pollObj = select.poll()
pollObj.register(teenSer,select.POLLIN) # Checks if data to read

# Byte arrays to store incoming data
numChars = 16
recievedChars = bytearray(numChars)
newData = False

# Vars for recieveWithStartEndMarkers func
recvInProgress = False
ndx = 0
startMarker = b'<'
endMarker = b'>'
rc = b''

# ----------------------------------------------------
# Joystick Code --------------------------------------

# JS state storage
axis_states = {}
button_states = {}

# These constants were borrowed from linux/input.h
axis_names = {
    0x00 : 'x',
    0x01 : 'y',
    0x02 : 'z',
    0x03 : 'rx',
    0x04 : 'ry',
    0x05 : 'rz',
    0x06 : 'trottle',
    0x07 : 'rudder',
    0x08 : 'wheel',
    0x09 : 'gas',
    0x0a : 'brake',
    0x10 : 'hat0x',
    0x11 : 'hat0y',
    0x12 : 'hat1x',
    0x13 : 'hat1y',
    0x14 : 'hat2x',
    0x15 : 'hat2y',
    0x16 : 'hat3x',
    0x17 : 'hat3y',
    0x18 : 'pressure',
    0x19 : 'distance',
    0x1a : 'tilt_x',
    0x1b : 'tilt_y',
    0x1c : 'tool_width',
    0x20 : 'volume',
    0x28 : 'misc',
}

button_names = {
    0x120 : 'trigger',
    0x121 : 'thumb',
    0x122 : 'thumb2',
    0x123 : 'top',
    0x124 : 'top2',
    0x125 : 'pinkie',
    0x126 : 'base',
    0x127 : 'base2',
    0x128 : 'base3',
    0x129 : 'base4',
    0x12a : 'base5',
    0x12b : 'base6',
    0x12f : 'dead',
    0x130 : 'a',
    0x131 : 'b',
    0x132 : 'c',
    0x133 : 'x',
    0x134 : 'y',
    0x135 : 'z',
    0x136 : 'tl',
    0x137 : 'tr',
    0x138 : 'tl2',
    0x139 : 'tr2',
    0x13a : 'select',
    0x13b : 'start',
    0x13c : 'mode',
    0x13d : 'thumbl',
    0x13e : 'thumbr',

    0x220 : 'dpad_up',
    0x221 : 'dpad_down',
    0x222 : 'dpad_left',
    0x223 : 'dpad_right',

    # XBox 360 controller uses these codes.
    0x2c0 : 'dpad_left',
    0x2c1 : 'dpad_right',
    0x2c2 : 'dpad_up',
    0x2c3 : 'dpad_down',
}

axis_map = []
button_map = []

# Open the joystick device.
fn = '/dev/input/js0'
print('Opening %s...' % fn)

try:
    jsdev = open(fn, 'rb')

except IOError:
    print('No PS3 Controller connected')
    print('Please press the PS button to connect...')

    while True:
        if os.path.exists('/dev/input/js0'):
            print('Controller connected')

            jsdev = open(fn, 'rb')
            break

# Get the device name.
buf = bytearray(63)
# buf = array.array('u', ['\0'] * 64)
ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
# Get rid of random padding
buf = buf.rstrip(b'\0')
js_name = str(buf, encoding='utf-8')
print('Device name: %s' % js_name)

# Get number of axes and buttons.
buf = array.array('B', [0])
ioctl(jsdev, 0x80016a11, buf) # JSIOCGAXES
num_axes = buf[0]

buf = array.array('B', [0])
ioctl(jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
num_buttons = buf[0]

# Get the axis map.
buf = array.array('B', [0] * 0x40)
ioctl(jsdev, 0x80406a32, buf) # JSIOCGAXMAP

for axis in buf[:num_axes]:
    axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
    axis_map.append(axis_name)
    axis_states[axis_name] = 0.0

# Get the button map.
buf = array.array('H', [0] * 200)
ioctl(jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

for btn in buf[:num_buttons]:
    btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
    button_map.append(btn_name)
    button_states[btn_name] = 0

# Report on the number of axes and buttons (Debug only)
# print('%d axes found: %s' % (num_axes, ', '.join(axis_map)))
# print('%d buttons found: %s' % (num_buttons, ', '.join(button_map)))

# --------------------------------------------------
# Read BNO055 function -----------------------------
def update_orientation(sampletime):
    global heading
    global roll
    global pitch
    global last_pitch
    global curr_time
    global prev_time
    global bno
    global my_drive
    global pitchrate

    # Get current time
    curr_time = time_.time()

    # Catch a possibly disconnected IMU (Safety)
    try:
        if (curr_time - prev_time) >= 1/float(sampletime):
            heading, roll, pitch = bno.read_euler()
            # print('IMU Read')
            # Calculate angular speed
            pitchrate = (pitch - last_pitch)/(curr_time - prev_time)

            last_pitch = pitch
            prev_time = curr_time

    except RuntimeError as sensorerror:
        print(sensorerror)
        print('Turning off motors...')
        my_drive.axis0.requested_state = AXIS_STATE_IDLE
        my_drive.axis1.requested_state = AXIS_STATE_IDLE
        setpt_on = False
        angle_cnt = 0

# --------------------------------------------------
# Convert encoder angles to relative deg -----------

def todeg(counts):
    mod = counts % 8192
    return float(mod)/8192*360 # Degrees

def tocounts(deg):
    return float(deg)/360*8192 # Counts

def torad(deg):
    return float(deg)/180*3.141592653589793 # Radians

def bindto360(deg):
    if deg < 360:
        return deg
    elif deg > 360:
        return deg - 360

def bindto180(deg):
    if deg < 180:
        return deg
    else:
        return deg - 360

def bindto18(deg):
    segment = deg % 36

    if segment <= 18:
        return segment
    elif segment > 18:
        return segment - 36

def m0UpdateAbsAngle():
    global m0pos_offset
    global my_drive
    global m0pos
    global m0pos_deg
    global m0theta
    global m0speed
    global pitch
    global m0beta

    # Get current position of encoder -> wheel (raw)
    m0pos = my_drive.axis0.encoder.pos_estimate/4.5

    # Convert to degrees (for DEBUG only)
    m0pos_deg = todeg(m0pos)

    # Get current position of wheel relative to the zeroed position
    theta = todeg(m0pos - m0pos_offset) #(wheel angle)
    # theta = todeg(-m0pos) + m0pos_offset

    # Get speed (for data only)
    m0speed_raw = my_drive.axis0.encoder.vel_estimate

    # Convert speed to deg/s from counts/s
    m0speed = m0speed_raw/8192*360 # Deg/s
    m0wspeed = torad(m0speed/4.5) # Rad/s

    # Get current pitch of body
    # heading, roll, pitch = bno.read_euler()

    # Find absolute position of the wheel relative to the ground - based on
    # the marked spoke and the absolute position offset
    # Then bind the absolute position to 360 degrees then to +/- 180 degrees
    # beta_diff = pitch - theta
    m0beta = bindto180(bindto360(pitch + theta + beta0)) ######

def m1UpdateAbsAngle():
    global m1pos_offset
    global my_drive
    global m1pos
    global m1pos_deg
    global m1theta
    global m1speed
    global pitch
    global m1beta

    # Get current position of encoder -> wheel (raw)
    m1pos = my_drive.axis1.encoder.pos_estimate/4.5

    # Convert to degrees (for DEBUG only)
    m1pos_deg = todeg(m1pos)

    # Get current position of wheel relative to the zeroed position
    m1theta = todeg(-m1pos + m1pos_offset) #(wheel angle)
    # theta = todeg(-m1pos) + m1pos_offset

    # Get speed (for data only)
    m1speed_raw = my_drive.axis1.encoder.vel_estimate

    # Convert speed to deg/s from counts/s
    m1speed = m1speed_raw/8192*360 # Deg/s
    m1wspeed = torad(m1speed/4.5) # Rad/s

    # Get current pitch of body
    # heading, roll, pitch = bno.read_euler()

    # Find absolute position of the wheel relative to the ground - based on
    # the marked spoke and the absolute position offset
    # Then bind the absolute position to 360 degrees then to +/- 180 degrees
    # beta = pitch - theta
    m1beta = bindto180(bindto360(pitch + m1theta + beta0)) ######


# ---------------------------------------------------
# Communicate with Arduino ----------------------------

def ardRecieve():
    global mapped_out

    # Poll object checks if there is data to be read from USB at /dev/ACM* 
    if pollObj.poll(0.05):
        ardIn_b = ardSer.readline()
        
        try:
            ardIn = ardIn_b.decode("utf-8") 
            # print(ardIn)
            if ardIn:
                # Take the read line and strip the \n and the <>
                mapped_out = float(ardIn[:-2].strip('<').strip('>'))
                # print(mapped_out)
        except (ValueError) as VE:
            print(VE)

# def ardRecieve():
#     global mapped_out
#     global newData
#     global recievedChars

#     # If there is data in the input buffer read a line
#     try:
#         recieveWithStartEndMarkers()
#         # print(recievedChars)
#         newData = False
#         ardInArr = recievedChars.split(b'\0',1)[0]
#         ardIn = ardInArr.decode()
#         mapped_out = float(ardIn)
#         # print(mapped_out)
#     except ValueError as VE:
#         print(VE)
#         newData = False

def ardSend():
    global feedback
    global angle_cnt
    global setpt_on
    global curr_time_ard
    global prev_time_ard

    # Get current time
    curr_time_ard = time_.time()

    # Send to Arduino at 100Hz
    if (curr_time_ard-prev_time_ard) >= .01:
        ardSer.write(b'<%.4f,%.2f,%i,%.4f>' % (feedback,angle_cnt,setpt_on,-2.0*axis_states['x']))
        prev_time_ard = curr_time_ard

def recieveWithStartEndMarkers():
    global recvInProgress
    global ndx
    global rc

    global recievedChars
    global newData

    global ardSer

    # Clear vars
    # recievedChars = bytearray(numChars)
    recvInProgress = False
    # rc = b''

    while (ardSer.in_waiting > 0) and (newData == False):
        rc = ardSer.read()
        # print(rc)
        if recvInProgress:
            if rc != endMarker:
                # print(ndx)
                recievedChars[ndx] = ord(rc)
                ndx+=1
                # print(ndx)
                if ndx >= numChars:
                    ndx = numChars - 1
            else:
                recievedChars[ndx] = ord(b'\0')
                recvInProgress = False
                ndx = 0
                newData = True

        elif rc == startMarker:
            recvInProgress = True
            # print('Recieve in Progress')

# ---------------------------------------------------
# Controller (Aykut & Wankun) -----------------------
# def phi(k,theta):
#     global theta

# ---------------------------------------------------
# Read Joystick Function ----------------------------

def readJS():
    global armed
    global setpt_on
    global angle_cnt
    global calibrating
    global my_drive
    global jsdev

    while True:
        # Read the joystick
        try:
            evbuf = jsdev.read(8)

        # If the controller disconnects during operation, turn off motors and wait for reconnect
        except IOError:
            my_drive.axis0.requested_state = AXIS_STATE_IDLE
            my_drive.axis1.requested_state = AXIS_STATE_IDLE

            print('No PS3 Controller connected')
            print('Please press the PS button to connect...')

            while True:
                if os.path.exists('/dev/input/js0'):
                    print('Controller connected')

                    evbuf = jsdev.read(8)
                    break

        if evbuf:
            time, value, type, number = struct.unpack('IhBB', evbuf)

            # Determine if evbuf is the initial value
            if type & 0x80:
                # print((initial)),
                continue

            # Determine if evbuf is a button
            if type & 0x01:
                button = button_map[number]
                if button:
                    button_states[button] = value
                    # Print states (debug only)
                    # print(value)
                    # if value:
                    #     print("%s pressed" % (button))
                    # else:
                    #     print("%s released" % (button))

            # Determine if evbuf is an axis
            if type & 0x02:
                axis = axis_map[number]
                if axis:
                    fvalue = value / 32767.0
                    axis_states[axis] = fvalue
                    # Print states (debug only)
                    # print("%s: %.3f" % (axis, fvalue))

            # Start shutdown procedure
            if button_states['tr'] and button_states['tl'] and button_states['start']:
                print('Shutting down...')
                call('sudo shutdown --poweroff now', shell=True)

            # Calibrate (Arm) the ODrive if the start button is pressed
            if button_states['start'] and (armed == False):
                # Calibrate motor and wait for it to finish
                if not my_drive.axis0.motor.is_calibrated:
                    print("Calibrating M0...")
                    calibrating = True
                    my_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

                if not my_drive.axis1.motor.is_calibrated:
                    print("Calibrating M1...")
                    calibrating = True
                    my_drive.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

                if calibrating:
                    while (my_drive.axis0.current_state != AXIS_STATE_IDLE) or (my_drive.axis1.current_state != AXIS_STATE_IDLE):
                        time_.sleep(0.1)

                my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                my_drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

                my_drive.axis0.controller.current_setpoint = 0
                my_drive.axis1.controller.current_setpoint = 0

                my_drive.axis0.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
                my_drive.axis1.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL

                armed = True

                print("Motors Ready!")

                continue

            # Turn off motor if the select button is pressed
            if button_states['select']:
                print('Turning off motors...')
                my_drive.axis0.requested_state = AXIS_STATE_IDLE
                my_drive.axis1.requested_state = AXIS_STATE_IDLE
                setpt_on = False
                angle_cnt = 0
                continue

            # Increase angular setpoint if up is pressed
            if button_states['dpad_up']:
                if armed:
                    if setpt_on:
                        if angle_cnt < 50:
                            angle_cnt += 5
                            print('Setpoint is %d degrees' % angle_cnt)
                    else:
                        setpt_on = True
                        print('setpt_on is True')
                    # time_.sleep(0.05)   # Debounce

            # Decrease angular setpoint if down is pressed
            if button_states['dpad_down']:
                if armed:
                    if setpt_on:
                        angle_cnt -= 5
                        if angle_cnt <= 0:
                            angle_cnt = 0
                            setpt_on = False
                            continue

                        print('Setpoint is %d degrees' % angle_cnt)
                        # time_.sleep(0.05)   # Debounce

def sendReadData(pipe):
    try:
        # pt = time_.time()

        ## Read from the pipe; this will be spawned as a separate Process
        p_out, p_in = pipe
        p_in.close()    # Only reading

        # prev_time = time_.time()

        # Open txt file to temporarily save test data and then write PID gains
        testdata = open('testdata.csv','w')
        testdata.write('0.17, 0.0005, 16.0\n')

        while True:
            if p_out.poll():
                data_out = p_out.recv()
                testdata.write('%.6f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n' % (data_out['curr_time'], data_out['feedback'], data_out['angle_cnt'], data_out['mapped_out'], data_out['pitchrate'], data_out['m1pos_deg'], data_out['m1speed'], data_out['m1beta'], data_out['vbus'], data_out['m1curr'], data_out['m0pos_deg'],data_out['m0speed']))
                # ct = time_.time()
                # if (ct - pt) >= .1:
                    # print('Pitch: %.4f\tSetpoint: %.4f Deg' % (data_out['pitch'], data_out['angle_cnt']))
                    # print('Pitch: %.4f    Feedback: %.4f    Setpoint: %.4f Deg    Mapped PID Output: %.4f' % (data_out['pitch'], data_out['feedback'], data_out['angle_cnt'], data_out['mapped_out']))
                    # pt = ct     
    finally:
        testdata.close()

if __name__ == '__main__':

    # Start the readJS thread. Reads joystick in the "background"
    readJSThrd = threading.Thread(target=readJS)
    readJSThrd.daemon = True
    readJSThrd.start()

    # Start the sendReadData process. Writes and sends data outside the main loop
    p_out, p_in = Pipe()
    sendReadDataThrd = Process(target=sendReadData, args=((p_out,p_in,),))
    sendReadDataThrd.daemon = True
    sendReadDataThrd.start()

    p_out.close() # Don't need this anymore

    # Open txt file to temporarily save test data and then write PID gains
    # testdata = open('testdata.csv','w')
    # testdata.write('%.2f, %.2f, %.2f\n' % (kp, ki, kd))
    # testdata.write('curr_time, feedback, angle_cnt, mapped_out, pitchrate, m1pos_deg, m1speed, m1beta, vbus, m1curr\n')

    pt = time_.time()

    # ----------------------------------------------------
    # Loop -----------------------------------------------
    while True:
        # Allows ctrl-C to exit the program, should keep the IMU stable
        try:
            # Speed check the motors -- Safety precaution
            ct = time_.time()
            if setpt_on:
                if (abs(my_drive.axis0.encoder.vel_estimate) > vel_limit) or (abs(my_drive.axis1.encoder.vel_estimate) > vel_limit) :
                    # Shutdown motors
                    print('Max speed reached, turning off motors...')
                    my_drive.axis0.requested_state = AXIS_STATE_IDLE
                    my_drive.axis1.requested_state = AXIS_STATE_IDLE
                    setpt_on = False
                    angle_cnt = 0

            # Update IMU at 100 Hz
            update_orientation(100)

            # Bound pitch values (for feedback) to 0 - 180 deg
            if pitch >= 0.0:
                feedback = pitch
            else:
                feedback = 0.0

            # Get data from ODrive
            m0UpdateAbsAngle()
            m1UpdateAbsAngle()
            # m1curr = my_drive.axis1.motor.current_control.Iq_measured
            m1curr = my_drive.axis1.motor.current_control.Ibus
            vbus = my_drive.vbus_voltage

            # Get/send data from Arduino
            ardRecieve()
            ardSend()

            # if curr_time-prev_time >= .05:
            # dtime = ct-pt
            # data_out = {'curr_time':curr_time, 'pitch':pitch, 'feedback':feedback, 'angle_cnt':angle_cnt, 'mapped_out':mapped_out, 'pitchrate':pitchrate, 'm1pos_deg':m1pos_deg, 'm1speed':m1speed, 'm1beta':m1beta, 'vbus':vbus, 'm1curr':m1curr}
            # print(data_out)
            # p_in.send(data_out)
            # print('written')
            p_in.send({'curr_time':curr_time, 'pitch':pitch, 'feedback':feedback, 'angle_cnt':angle_cnt, 'mapped_out':mapped_out, 'pitchrate':pitchrate, 'm1pos_deg':m1pos_deg, 'm1speed':m1speed, 'm1beta':m1beta, 'vbus':vbus, 'm1curr':m1curr, 'diffval':-2.0*axis_states['x'],'m0pos_deg':m0pos_deg,'m0speed':m0speed})
            
            if (ct-pt) > 0.1:
                print('Pitch: %.4f\tSetpoint: %.4f Deg\tDiff: %.4f' % (pitch, angle_cnt,1.5*axis_states['x']))
                pt = ct

        except (KeyboardInterrupt):
            # Turn off the motors
            my_drive.axis0.requested_state = AXIS_STATE_IDLE
            my_drive.axis1.requested_state = AXIS_STATE_IDLE
            # Close the data file
            setpt_on = False
            ardSer.flush()
            ardSer.write(b'<%.4f,%.2f,%i>' % (feedback,angle_cnt,setpt_on))
            ardSer.close()
            sys.exit()
