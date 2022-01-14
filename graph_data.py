#sensor fusion in my home directory
from mbientlab.metawear import MetaWear, libmetawear, parse_value
from mbientlab.metawear.cbindings import *
import time
import math
import matplotlib.pyplot as plt
import scipy.integrate
import numpy
import sys, getopt
from mpl_toolkits.mplot3d import Axes3D
from scipy import signal

def function(do_acc, do_gyro):
    # which sensor are we using
    #address = "C7:EA:21:57:F5:E2" #Will
    address = "C4:A3:A4:75:A2:86" #Matt

    device = MetaWear(address)
    device.connect()
    session = State(device)
    if do_acc == 1:
        session.startup_accelerometer()
    if do_gyro == 1:
        session.startup_gyroscope()
    if do_acc == 1:
        session.enable_accelerometer()
    if do_gyro == 1:
        session.enable_gyroscope()
    
    print("Recording")
    time.sleep(5) #how long the sensor will record for
    print("Stopped")
    if do_acc == 1:
        session.shutdown_accelerometer()
    if do_gyro == 1:
        session.shutdown_gyroscope()

    device.disconnect()


    if do_acc == 1 and do_gyro == 1:
        session.plot_acc_gyr()
    elif do_acc == 1:
        session.plot_acc()
    elif do_gyro == 1:
        session.plot_gyr()

class State():
    def __init__(self, device):
        self.device = device
        self.acc_samples = 0
        self.gyr_samples = 0
        self.acc_x = []
        self.acc_y = []
        self.acc_z = []

        self.gyr_x = []
        self.gyr_y = []
        self.gyr_z = []

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

    def startup_gyroscope(self):
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
        self.acc_x.append(d.x)
        self.acc_y.append(d.y)
        self.acc_z.append(d.z)
        self.acc_samples += 1
    
    def gyr_data_handler (self, ctx, data):
        d = parse_value(data)
        self.gyr_x.append(d.x)
        self.gyr_y.append(d.y)
        self.gyr_z.append(d.z)
        self.gyr_samples += 1

    def plot_acc (self):
        
        plt.plot(self.acc_x, label = "X")
        plt.plot(self.acc_y, label = "Y")
        plt.plot(self.acc_z, label = "Z")
        plt.title('Acc')
        plt.legend()
        plt.show()

    def plot_gyr (self):
        plt.plot(self.gyr_x, label = "X")
        plt.plot(self.gyr_y, label = "Y")
        plt.plot( self.gyr_z, label = "Z")
        plt.title('Gyr')
        plt.legend()
        plt.show()

    def plot_acc_gyr(self):

        figs, axs = plt.subplots(1,2)
        axs[0].plot(self.acc_x, label = "X")
        axs[0].plot(self.acc_y, label = "Y")
        axs[0].plot(self.acc_z, label = "Z")
        axs[0].set_title('Acc')
        axs[0].legend()

        axs[1].plot(self.gyr_x, label = "X")
        axs[1].plot(self.gyr_y, label = "Y")
        axs[1].plot(self.gyr_z, label = "Z")
        axs[1].set_title('Gyr')
        axs[1].legend()
        
        plt.show()

def main(argv):    
    do_acc = 0
    do_gyro = 0
    
    try:
        opts, args = getopt.getopt(argv,"ag",["acc","gyr"])
    except getopt.GetoptError:
        print ("error FUCK")
        sys.exit(2)
    for opt, arg in opts:
        if opt in ("-a","--acc"):
            print ('Finna do accecleration')
            do_acc = 1
        elif opt in ("-g", "--gyr"):
            print ('Finna do gyroscope')
            do_gyro = 1

    print(do_acc,do_gyro)
    function(do_acc, do_gyro)

if __name__ == "__main__":
    main(sys.argv[1:])

