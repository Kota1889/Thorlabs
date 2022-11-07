"""An example that uses the .NET Kinesis Libraries to connect to a KDC."""
import os
import time
import sys
import clr

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

def main():
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
    meter.setWavelength(c_double(1064.0))

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

        m_config.DeviceSettingsName = str('PRM1-Z8')
        m_config.UpdateCurrentConfiguration()
        kcube.SetSettings(kcube.MotorDeviceSettings, True, False)

        # Home stage
        print("Homing Device...")
        kcube.Home(60000)  # 60 second timeout
        print("Device Homed")

        jog_params = kcube.GetJogParams()
        jog_params.StepSize = Decimal(180) #1 degree stepsize
        jog_params.MaxVelocity = Decimal(30)
        jog_params.JogMode = JogParametersBase.JogModes.SingleStep

        kcube.SetJogParams(jog_params)

        print('Moving Motor')
        kcube.MoveJog(MotorDirection.Forward,0)
        time.sleep(.25)

        # Stop polling and close device
        while kcube.IsDeviceBusy:
            power = c_double()
            meter.measPower(byref(power))
            print(f'{kcube.Position},{power.value*1000}')
            time.sleep(.1)

        kcube.StopPolling()
        kcube.Disconnect(False)
    meter.close()

if __name__ == "__main__":
    main()