# imu.py
# Author: Ethan Garnier
# Last Modified: January 21, 2023
import logging
import sys
import time

from Adafruit_BNO055 import BNO055
from datetime import datetime

# Raspberry Pi configuration with serial UART:
bno = BNO055.BNO055(serial_port='/dev/serial0')

def print_sys_info(logfile):
    """Print the BNO055 IMU system information"""
    # Print system status and self test result.
    status, self_test, error = bno.get_system_status()

    statusStr = "System status: {0}".format(status)
    print(statusStr)
    logfile.write(statusStr + '\n')

    selfTestStr = "Self test result (0x0F is normal): 0x{0:02X}".format(self_test)
    print(selfTestStr)
    logfile.write(selfTestStr)

    # Print out an error if system status is in error mode.
    if status == 0x01:
        errorStr = "System error: {0}\nSee datasheet section 4.3.59 for the meaning.".format(error)
        print(errorStr)
        logfile.write(errorStr)

    # Print BNO055 software revision and other diagnostic data.
    sw, bl, accel, mag, gyro = bno.get_revision()
    revisionStr = "Software version:   {0}\nBootloader version: {0}\nccelerometer ID:   0x{0:02X}\nMagnetometer ID:    0x{0:02X}\nGyroscope ID:       0x{0:02X}\n".format(
        sw, bl, accel, mag, gyro)
    print(revisionStr)
    logfile.write(revisionStr + '\n')

def calibrate_imu():
    """Instruct the user on how to manually calibrate
    the BNO055 IMU magenetometer, gyroscope, and accelerometer.
    Once each of these are calibrated, export the calibration
    offsets to calibration.json. This file may be used in the future
    to recalibrate the sensors in the event they become uncalibrated.
    """
    # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    sys, gyro, accel, mag = bno.get_calibration_status()

    print("\n-----------\n")
    print("Calibrating BNO055 Device:\n")
    
    print("\tTo calibrate the Gyroscope, place the device down and let rest still...\n") 
    while gyro != 3: sys, gyro, accel, mag = bno.get_calibration_status()
    print("\tDone calibrating Gyroscope\n")

    print("\tTo calibrate the Magnetometer, move the device through the air in a figure-8 pattern...\n")
    while mag != 3: sys, gyro, accel, mag = bno.get_calibration_status()
    print("\tDone calibrating Magnetometer\n")

    print("\tTo calibrate the Accelerometer, rotate the device around an axis in 45 degree increments...\n")
    while accel != 3: sys, gyro, accel, mag = bno.get_calibration_status()
    print("\tDone calibrating Accelerometer\n")

    while sys != 3: sys, gyro, accel, mag = bno.get_calibration_status()

    print("Calibration complete!\n")
    print("-----------\n\n")

    # TODO: Write calibration offsets to calibration.json

def run_imu():
    """Execute the main imu subroutine that will log
    the heading, roll, and pitch of the device to stdout and
    a log file passed as argument.
    """
    # Initialize the BNO055 and stop if something went wrong.
    if not bno.begin():
        raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')
    
    # Construct the logfile
    now = datetime.now()
    dt = now.strftime("%d-%m-%Y_%H-%M-%S")
    logfilename = f"BNO055_LOG_{dt}.txt"

    logfile = open(logfilename, "w")
    logfile.write(f"BNO055 Log File - {dt}\n")

    # Use this try block to ensure the log file is closed
    try:
        print_sys_info(logfile)
        calibrate_imu()

        print('Reading BNO055 data, press Ctrl-C to quit...')
        while True:
            # Read the Euler angles for heading, roll, pitch (all in degrees).
            heading, roll, pitch = bno.read_euler()
            # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
            sys, gyro, accel, mag = bno.get_calibration_status()
            # Construct a string full of measured data
            data = "Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}".format(
                heading, roll, pitch, sys, gyro, accel, mag)

            # Print data to stdout and log file
            print(data)
            logfile.write(data + '\n')

            # Other values you can optionally read:
            # Orientation as a quaternion:
            # x,y,z,w = bno.read_quaterion()
            # Sensor temperature in degrees Celsius:
            # temp_c = bno.read_temp()
            # Magnetometer data (in micro-Teslas):
            # x,y,z = bno.read_magnetometer()
            # Gyroscope data (in degrees per second):
            # x,y,z = bno.read_gyroscope()
            # Accelerometer data (in meters per second squared):
            # x,y,z = bno.read_accelerometer()
            # Linear acceleration data (i.e. acceleration from movement, not gravity--
            # returned in meters per second squared):
            # x,y,z = bno.read_linear_acceleration()
            # Gravity acceleration data (i.e. acceleration just from gravity--returned
            # in meters per second squared):
            # x,y,z = bno.read_gravity()
            # Sleep for a second until the next reading.
            time.sleep(1)
    except KeyboardInterrupt:
        logfile.close()

def parse_args():
    if len(sys.argv) > 2:
        for arg in sys.argv:
            if arg.lower() == '-v':
                logging.basicConfig(level=logging.DEBUG)
            elif arg.lower() == '-h':
                print("Usage: sudo python imu.py [-v -h]")

if __name__ == '__main__':
    # Parse command line arguments
    parse_args()

    # Run the main IMU routine
    run_imu()