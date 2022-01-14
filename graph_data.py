#sensor fusion in my home directory
from mbientlab.metawear import MetaWear, libmetawear, parse_value
from mbientlab.metawear.cbindings import *
import time
import math
import matplotlib.pyplot as plt
import scipy.integrate
import numpy
from mpl_toolkits.mplot3d import Axes3D
from scipy import signal

def main():
    # which sensor are we using
    #address = "C7:EA:21:57:F5:E2" #Will
    address = "C4:A3:A4:75:A2:86" #Matt

    device = MetaWear(address)
    device.connect()
    session = State(device)

    session.startup_accelerometer()
    session.startup_gyroscrope()

    session.enable_accelerometer()
    session.enable_gyroscope()
    
    print("Recording")
    time.sleep(5) #how long the sensor will record for
    print("Stopped")
    session.shutdown_accelerometer()
    session.shutdown_gyroscope()

    device.disconnect()

class State():
    def __init__(self, device):
        self.device = device
        self.acc_samples = 0
        self.gyr_samples = 0
        self.acc = []
        self.gyr = []
        self.acc_callback = FnVoid_VoidP_DataP(self.acc_data_handler)
        self.gyr_callback = FnVoid_VoidP_DataP(self.gyr_data_handler)
        self.signal_acc = 0
        self.signal_gyr = 0
    def startup_accelerometer(self):

        #Setup the accelerometer sample frequency and range
        libmetawear.mbl_mw_acc_set_odr(self.device.board, 100.0)
        libmetawear.mbl_mw_acc_set_range(self.device.board, 16.0)
        libmetawear.mbl_mw_acc_write_acceleration_config(self.device.board)

        #Get the accelerometer data signal
        self.signal_acc = libmetawear.mbl_mw_acc_get_acceleration_data_signal(self.device.board)
        
        #Subscribe to it
        libmetawear.mbl_mw_datasignal_subscribe(self.signal_acc, None, self.acc_callback)

    def enable_accelerometer(self):

        #Enable the accelerometer
        libmetawear.mbl_mw_acc_enable_acceleration_sampling(self.device.board)
        libmetawear.mbl_mw_acc_start(self.device.board)

    def startup_gyroscrope(self):
        #Get the gyroscope data signal
        self.signal_gyr = libmetawear.mbl_mw_gyro_bmi270_get_packed_rotation_data_signal(self.device.board)
        
        #Subscribe to it
        libmetawear.mbl_mw_datasignal_subscribe(self.signal_gyr, None, self.gyr_callback)

    def enable_gyroscope(self):
        
        #Enable the gyroscrope
        libmetawear.mbl_mw_gyro_bmi270_enable_rotation_sampling(self.device.board)
        libmetawear.mbl_mw_gyro_bmi270_start(self.device.board)

    def shutdown_accelerometer(self):

        #Disable the accelerometer
        libmetawear.mbl_mw_acc_stop(self.device.board)
        libmetawear.mbl_mw_acc_disable_acceleration_sampling(self.device.board)
        
        #Unsubscrive to accelerometer
        libmetawear.mbl_mw_datasignal_unsubscribe(self.signal_acc)
    
    def shutdown_gyroscope(self):
        #Disable the gyroscope
        libmetawear.mbl_mw_gyro_bmi270_stop(self.device.board)
        libmetawear.mbl_mw_gyro_bmi270_disable_rotation_sampling(self.device.board)

        #Unsubscribe to it
        libmetawear.mbl_mw_datasignal_unsubscribe(self.signal_gyr)

    def acc_data_handler (self, ctx, data):
        d = parse_value(data)
        self.acc.append([d.x, d.y, d.z])
        print(d.x)
        self.acc_samples += 1
    
    def gyr_data_handler (self, ctx, data):
        d = parse_value(data)
        self.gyr.append([d.x, d.y, d.z])
        print(d.x)
        self.gyr_samples += 1

if __name__ == "__main__":
    main()

