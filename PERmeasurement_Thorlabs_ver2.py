DESCRIPTION = """PER measurement using Thorlabs Power meter and Half-wave plate rotation.

Example
-------
# Preparation
    1. pip install pythonnet
    2. download Thorlabs software for Optical Power Meter and Rotation mount (KINESIS)
    3. create folder with PERmeasurement_Thorlabs.py, TLPM.py, TLPM_64.dll

# Run the data logging program in terminal
python PERmeasurement_Thorlabs.py -ss 180 -mv 30 -int 3 -cw 1064

# See help
Resource1: https://github.com/Thorlabs/Motion_Control_Examples
Resource2: https://www.youtube.com/watch?v=VbcCDI6Z6go&list=LL&index=1&t=600s

- Author :Kota Koike
- First draft : 2022/11/07
"""
import os
import time
import sys
import clr

from typing import Tuple
from time import sleep
import numpy as np
import struct
import datetime
import argparse

# Add References to .NET libraries
clr.AddReference("C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.DeviceManagerCLI.dll.")
clr.AddReference("C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.GenericMotorCLI.dll.")
clr.AddReference('C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.KCube.DCServoCLI.dll.')
clr.AddReference("C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.KCube.BrushlessMotorCLI.dll.")

from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import KCubeMotor
from Thorlabs.MotionControl.GenericMotorCLI.ControlParameters import JogParametersBase
from Thorlabs.MotionControl.KCube.DCServoCLI import *
from Thorlabs.MotionControl.KCube.BrushlessMotorCLI import *
from System import Decimal

from ctypes import cdll,c_long, c_ulong, c_uint32,byref,create_string_buffer,c_bool,c_char_p,c_int,c_int16,c_double, sizeof, c_voidp
from TLPM import TLPM

    # =========================================================
    # Settimg parameters
    # =========================================================
def set_jogparams(STEP_SIZE, MAX_VELOCITY):  
    #Add parameters below
    jog_params = kcube.GetJogParams()
    jog_params.StepSize = Decimal(STEP_SIZE) #1 degree stepsize
    jog_params.MaxVelocity = Decimal(MAX_VELOCITY)
    jog_params.JogMode = JogParametersBase.JogModes.SingleStep
    kcube.SetJogParams(jog_params)

# =========================================================
# Main program:
# =========================================================
VERSION = 0.2

if __name__ == '__main__':

    # argument parser
    # Add parameters below
    parser = argparse.ArgumentParser(description=DESCRIPTION, epilog='Version: {VERSION}')
    parser.add_argument('-ss', '--step_size', help='Step Size (degree)', type=float, default=180)
    parser.add_argument('-mv', '--max_velocity', help='Maximum Velocity (degree/sec)', type=float, default=30)
    parser.add_argument('-int', '--meas_interval', help='Measurement interval (s)', type=float, default=3)
    parser.add_argument('-cw', '--center_wavelength', help='Center wavelength (nm)', type=float, default=1064)

    args = parser.parse_args()

    STEP_SIZE = args.step_size
    MAX_VELOCITY = args.max_velocity
    CENTER_WAVELENGTH = args.center_wavelength
    MEA_INTERVAL =  args.meas_interval

    # =========================================================
    # Initialize Thorlabs PM100 Power Meter and KBD101 Brushless Motorlized Rotation Mount
    # =========================================================
    """The main entry point for the application"""
    os.add_dll_directory(os.getcwd())
    meter = TLPM()
    device_count = c_uint32()
    meter.findRsrc(byref(device_count))

    if device_count == 0:
        print('No connected meters')
        quit()
    
    resource_name = create_string_buffer(1024)
    meter.getRsrcName(c_int(0), resource_name)

    meter.open(resource_name, c_bool(True), c_bool(True))
    meter.setWavelength(c_double(CENTER_WAVELENGTH))

    # Build device list so that the library can find yours
    DeviceManagerCLI.BuildDeviceList()
    # create new device
    serial_no = str("28250836")  # Replace this line with your device's serial number
    kcube = KCubeBrushlessMotor.CreateKCubeBrushlessMotor(serial_no)

    if not kcube == None:
    # Connect, begin polling, and enable
        kcube.Connect(serial_no)

    # Wait for Settings to Initialise
        if not kcube.IsSettingsInitialized():
            kcube.WaitForSettingsInitialized(10000)  # 10 second timeout

        kcube.StartPolling(50)
        time.sleep(.1)  # wait statements are important to allow settings to be sent to the device
        kcube.EnableDevice()
        time.sleep(.1)  # Wait for device to enable

        # Get Device information
        device_info = kcube.GetDeviceInfo()
        print(device_info.Description)

        # Before homing or moving device, ensure the motors configuration is loaded
        m_config = kcube.LoadMotorConfiguration(serial_no,
                                                DeviceConfiguration.DeviceSettingsUseOptionType.UseDeviceSettings)

        m_config.DeviceSettingsName = str('DDR25/M') #name of motorized rotation mount
        m_config.UpdateCurrentConfiguration()
        kcube.SetSettings(kcube.MotorDeviceSettings, True, False)

        # Home stage
        print("Homing Device...")
        kcube.Home(60000)  # 60 second timeout
        print("Device Homed")
        print("Device Initialized")

    # =========================================================
    # Rotation parameters
    # =========================================================
    set_jogparams(STEP_SIZE, MAX_VELOCITY)
    # =========================================================
    # Data logging parameters
    # =========================================================
    i = 0 #current number of measurement
    file_name = "test" #file_name
    t_interval = MEA_INTERVAL #interval between measurement
    t_elapsed = 0 #curremt measuremet time (sec)
    t_elapsed_round = 0
    hour = 0 #curremt measuremet time (h)
    hour_round = 0

    max_power = 0 #mW, set max_power at 0 mW
    min_power = 1000 #mW, set min_power at 1000mW
    # =========================================================
    # Data logging start
    # =========================================================
    print("Start data logging ... (Press Ctrl+C to stop)\n")
    with open('PER_log.csv', 'w') as logfile:
        logfile.write("#PER measurement \n")
        logfile.write("#Start datetime:, %s\n" % datetime.datetime.now())
        logfile.write("#Center WL:, %e nm\n" % CENTER_WAVELENGTH)
        logfile.write("#Step Size:, %e degree\n" % STEP_SIZE)
        logfile.write("#Max Velocity:, %e degree/sec\n" % MAX_VELOCITY)
        logfile.write("#Measurement interval:, %e s\n" % MEA_INTERVAL)
        logfile.write("Time_elapsed (h), PER (dB) \n")

        try:
            while True:
                t1 = time.time() #sec

                t_elapsed_round = round(t_elapsed,3)
                #convert sec to hour
                hour = t_elapsed/60**2
                hour = round(hour,3)
                #経過時間表示
                print(f"Elapsed time: {t_elapsed_round} s, ({hour} h)")

                # Move stage from 0 to 180 degree
                print('Start measurement')
                kcube.MoveJog(MotorDirection.Forward,0)
                time.sleep(.25)

                # Record degree and power until stop at 180 degree
                while kcube.IsDeviceBusy:
                    current_power = c_double()
                    meter.measPower(byref(current_power))
                    #print(f'{kcube.Position},{current_power.value*1000}') #degree, mW
                    max_power = max(current_power.value, max_power) # get maximum power
                    min_power = min(current_power.value, min_power) # get minimum power
                    time.sleep(.1)

                # PER calculation
                PER = np.log10(max_power/min_power).round(3)
                print(f"PER is {PER} dB")

                # Write the PER value in csv file along with time
                logfile.write(f"{hour},{PER}\n")

                # Home stage
                print("Homing Device...")
                kcube.Home(60000)  # 60 second timeout
                print("Device Homed")

                # Wait for next measurement
                time.sleep(t_interval) #sec
                t2 = time.time() #sec
                t_elapsed += t2 - t1

        except KeyboardInterrupt:
            print("Measurement stoped at %.3f s\n" %(t_elapsed))
        
    kcube.StopPolling()
    kcube.Disconnect(False)
    meter.close()
# =========================================================
# Close program
# =========================================================
sys.exit()
