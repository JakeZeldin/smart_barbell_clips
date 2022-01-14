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

def function(do_acc, do_gyro,m,w, both_sens):
    #which sensor are we using
    if both_sens == 0 and w == 1:
        address = "C7:EA:21:57:F5:E2" #Will
    elif both_sens ==0 and m == 1:
        address = "C4:A3:A4:75:A2:86" #Matt
    else:
        address1 = "C4:A3:A4:75:A2:86" #Matt
        address2 = "C7:EA:21:57:F5:E2" #Will
    
    if both_sens == 0:
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
            if m ==1:
                session.plot_acc_gyr(1)
            else:
                session.plot_acc_gyr(2)
        elif do_acc == 1:
            if m == 1:
                session.plot_acc(1)
            else:
                session.plot_acc(2)
        elif do_gyro == 1:
            if w == 1:
                session.plot_gyr(1)
            else:
                session.plot_gyr(2)
    
    else:
        device1 = MetaWear(address)
        device2 = MetaWear(address)
        device1.connect()
        device2.connect()
        session1 = State(device1)
        session2 = State(device2)
        if do_acc == 1:
            session1.startup_accelerometer()
            session2.startup_accelerometer()
        if do_gyro == 1:
            session1.startup_gyroscope()
            session2.startup_gyroscope()
        if do_acc == 1:
            session1.enable_accelerometer()
            session2.enable_accelerometer()
        if do_gyro == 1:
            session1.enable_gyroscope()
            session2.enable_gyroscope()

        print("Recording")
        time.sleep(5) #how long the sensor will record for
        print("Stopped")
        if do_acc == 1:
            session1.shutdown_accelerometer()
            session2.shutdown_accelerometer()
        if do_gyro == 1:
            session1.shutdown_gyroscope()
            session2.shutdown_gyroscope()

        device1.disconnect()
        device2.disconnect()

        if do_acc == 1 and do_gyro == 1:
            session1.plot_acc_gyr(1)
            session2.plot_acc_gyr(2)
        elif do_acc == 1:
            session1.plot_acc(1)
            session2.plot_acc(2)
        elif do_gyro == 1:
            session1.plot_gyr(1)
            session2.plot_gyr(2)
    

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

    def plot_acc (self, num):
        if num == 1:
            sensor_name = "C4"
        else:
            sensor_name = "C7"
        plt.plot(self.acc_x, label = "X")
        plt.plot(self.acc_y, label = "Y")
        plt.plot(self.acc_z, label = "Z")
        plt.title('Acc ' + sensor_name)
        plt.legend()
        plt.show()

    def plot_gyr (self, num):
        if num == 1:
            sensor_name = "C4"
        else:
            sensor_name = "C7"
        plt.plot(self.gyr_x, label = "X")
        plt.plot(self.gyr_y, label = "Y")
        plt.plot( self.gyr_z, label = "Z")
        plt.title('Gyr ' + sensor_name)
        plt.legend()
        plt.show()

    def plot_acc_gyr(self, num):
        if num == 1:
            sensor_name = "C4"
        else:
            sensor_name = "C7"

        figs, axs = plt.subplots(1,2)
        axs[0].plot(self.acc_x, label = "X")
        axs[0].plot(self.acc_y, label = "Y")
        axs[0].plot(self.acc_z, label = "Z")
        axs[0].set_title('Acc ' + sensor_name)
        axs[0].legend()

        axs[1].plot(self.gyr_x, label = "X")
        axs[1].plot(self.gyr_y, label = "Y")
        axs[1].plot(self.gyr_z, label = "Z")
        axs[1].set_title('Gyr ' + sensor_name)
        axs[1].legend()
        
        plt.show()

def main(argv):    
    do_acc = 0
    do_gyro = 0
    m = 0
    w = 0
    both_sens = 0
    try:
        opts, args = getopt.getopt(argv,"agmw",["acc","gyr"])
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
        elif opt == "-m":
            m = 1
        elif opt == "-w":
            w = 1
    
    if m == 1 and w == 1:
        both_sens = 1
    
    if m ==0 and w == 0:
        print("No sensor selected use -m(c4) or -w(c7) to select which sensor your using")
        sys.exit(2)
    
    
    function(do_acc, do_gyro, m, w, both_sens)

if __name__ == "__main__":
    main(sys.argv[1:])

