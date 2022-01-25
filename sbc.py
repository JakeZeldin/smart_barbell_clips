from mbientlab.metawear import MetaWear, libmetawear, parse_value
from mbientlab.metawear.cbindings import *

import time
import matplotlib.pyplot as plt
import numpy as np
import scipy.integrate
import sys
import argparse
import csv
import os

# for orienting in global frame
from ahrs.filters import Madgwick
from ahrs import Quaternion


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("-r", nargs='+', 
            help="record with MAC address(es) of sensor(s) (only specify first"
            " two characters) default = C7",
            metavar="MAC")

    parser.add_argument("-s", nargs='+',
            help="save recording or loaded data to .data/CSV",
            metavar="CSV")
    parser.add_argument("-l", nargs='+',
            help="load linear acceleration from .data/CSV",
            metavar="CSV")
    parser.add_argument("-d", nargs='+',
            help="have first sensor enetered perform mbient fusion and second sensor perform wills fusion",
            metavar="CSV")
    parser.add_argument("-a", nargs='+',
            choices=['x', 'y', 'z'],
            help="plot acceleration in X, Y, and/or Z axis")
    parser.add_argument("-v", nargs='+',
            choices=['x', 'y', 'z'],
            help="plot velocity in X, Y, and/or Z axis")
    parser.add_argument("-p", nargs='+',
            choices=['x', 'y', 'z'],
            help="plot position in X, Y, and/or Z axis")

    parser.add_argument("-f", action="store_true",
            help="use mbient sensor fusion algorithm for absolute orientation "
            "acceleration")
    parser.add_argument("-c", action="store_true",
            help="correct raw acceleration with orientations from "
            "non-proprietary sensor fusion. Has no effect on mbient fusion")
    parser.add_argument("-t", default='5', type=int,
            help="time in T seconds to record for, default 5")

    # parser.add_argument("-z", "--zupt")
    
    args = parser.parse_args()

    if args.l is None and args.r is None:
        print("Nothing to do")
        return
    
    if len(args.r) != 2 and args.d is not None:
        print("-d selected but not 2 sensors entered")
        return 

    states = []
    
    #For when both sensors are performing different fusion opeations
    C7 = None 
    C4 = None

    if args.r is not None:
        macs = []
        #counter used to see what sensor was entered first for when -d flag set
        counter = 0;
        for mac in args.r:
            if mac == "C7":
                macs.append("C7:EA:21:57:F5:E2")
                if args.d is not None:
                    if counter == 0:
                        #c7 to perform mbient fusion c4 to perform will fusion
                        C7 = "mfus"
                        C4 = "wfus"
                    else:
                        #c7 to perform will fusion c4 to perform mbient fusion
                        C7 = "wfus"
                        C4 = "mfus"

            elif mac == "C4":
                macs.append("C4:A3:A4:75:A2:86")
                counter = 1

        for mac in macs:
            device = MetaWear(mac)
            device.connect()
            states.append(State(device,None,mac[:2]))
        
        if args.d is None:
            for s in states:
                if args.f:
                    s.start_fusion()
                else:
                    s.start_raw()
        else:
            for s in states:
                if s.mac == "C4":
                    #if for -d c4 to perform mbient fusion
                    if C4 == "mfus":
                        s.start_fusion()
                    #if for -d c7 to pergorm will fusion 
                    else:
                        s.start_raw()
                else:
                    #if for -d c7 to perform mbient fusion
                    if C7 == "mfus":
                        s.start_fusion()
                    #if for -d c7 to perform will fusion
                    else:
                        s.start_raw()

        time.sleep(args.t)
        
        if args.d is None:
            for i,s in enumerate(states):
                if args.f:
                    s.shutdown_fusion()
                else:
                    s.shutdown_raw()

                if args.c and not args.f:
                    s.conv_to_lin_acc(correct=True)
                else:
                    s.conv_to_lin_acc()

                if args.s is not None:
                    s.save_lin_acc(args.s[i])
        else:
            for s in states:
                if s.mac == "C4":
                    #if for -d c4 shutdown mbient fusion
                    if C4 == "mfus":
                        s.shutdown_fusion()
                        s.conv_to_lin_acc()
                    #if dor -d c4 shutdown raw data collection
                    else:
                        s.shutdown_raw()
                        s.conv_to_lin_acc(correct=True)
                if s.mac =="C7":
                    #if for -d c7 shutdown mbient fusion
                    if C7 == "mfus":
                        s.shutdown_fusion()
                        s.conv_to_lin_acc()
                    #if for -d c4 shutdown raw data collection 
                    else:
                        s.shutdown_raw()
                        s.conv_to_lin_acc(correct=True)


    if args.l is not None:
        for file_name in args.l:
            states.append(State(None, file_name))

    for s in states:
        s.calc_vel()
        s.calc_pos()
        np.set_printoptions(suppress=True, precision=3)
        print(s.lin_acc)
        print(s.vel)
        print(s.pos)


class State():
    def __init__(self, device, filename=None, mac=None):
        self.device = device
        self.acc = []
        self.gyr = []
        self.mac = mac

        if filename is None:
            self.lin_acc = None 

        else:
            try: 
                f = open(os.path.join(".data", filename), 'r')
            except OSError:
                print("Could not open data file")
                sys.exit(2)

            lin_acc = []
            reader = csv.reader(f)
            for row in reader:
                lin_acc.append([float(x) for x in row])
            self.lin_acc = np.array(lin_acc)
            f.close()

        self.vel = None
        self.pos = None

        self.acc_callback = FnVoid_VoidP_DataP(self.acc_data_handler)
        self.gyr_callback = FnVoid_VoidP_DataP(self.gyr_data_handler)

        self.signal_acc = None
        self.signal_gyr = None
        self.signal_fus = None


    def start_fusion(self):

        # Sensor fusion setup
        libmetawear.mbl_mw_sensor_fusion_set_mode(self.device.board, 
                SensorFusionMode.IMU_PLUS);
        libmetawear.mbl_mw_sensor_fusion_set_acc_range(self.device.board, 
                SensorFusionAccRange._8G);
        libmetawear.mbl_mw_sensor_fusion_set_gyro_range(self.device.board, 
                SensorFusionGyroRange._2000DPS);
        libmetawear.mbl_mw_sensor_fusion_write_config(self.device.board);

        # Subscribe to the linear acceleration signal
        signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(self.device.board, 
                SensorFusionData.CORRECTED_ACC);
        libmetawear.mbl_mw_datasignal_subscribe(signal, None, self.acc_callback);

        # Start sensor fusion (acc + gyro + on-board sensor fusion algo)
        libmetawear.mbl_mw_sensor_fusion_enable_data(self.device.board, 
                SensorFusionData.CORRECTED_ACC);
        libmetawear.mbl_mw_sensor_fusion_start(self.device.board);


    def start_raw(self):

        libmetawear.mbl_mw_acc_bmi160_set_odr(self.device.board,
                AccBmi160Odr._100Hz) # BMI 160 specific call
        libmetawear.mbl_mw_acc_bosch_set_range(self.device.board, AccBoschRange._4G)
        libmetawear.mbl_mw_acc_write_acceleration_config(self.device.board)

        # config gyro
        libmetawear.mbl_mw_gyro_bmi160_set_range(self.device.board, GyroBoschRange._1000dps);
        libmetawear.mbl_mw_gyro_bmi160_set_odr(self.device.board,
                GyroBoschOdr._100Hz);
        libmetawear.mbl_mw_gyro_bmi160_write_config(self.device.board);

        # get acc signal and subscribe
        acc = libmetawear.mbl_mw_acc_get_acceleration_data_signal(self.device.board)
        libmetawear.mbl_mw_datasignal_subscribe(acc, None, self.acc_callback)

        # get gyro signal and subscribe
        gyro = libmetawear.mbl_mw_gyro_bmi160_get_rotation_data_signal(self.device.board)
        libmetawear.mbl_mw_datasignal_subscribe(gyro, None, self.gyr_callback)

        # start acc
        libmetawear.mbl_mw_acc_enable_acceleration_sampling(self.device.board)
        libmetawear.mbl_mw_acc_start(self.device.board)

        # start gyro
        libmetawear.mbl_mw_gyro_bmi160_enable_rotation_sampling(self.device.board)
        libmetawear.mbl_mw_gyro_bmi160_start(self.device.board)


    def shutdown_fusion(self):
        # Stop sensor fusion
        libmetawear.mbl_mw_sensor_fusion_stop(self.device.board);

        # Unsubscribe
        signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(self.device.board, 
                SensorFusionData.CORRECTED_ACC);
        libmetawear.mbl_mw_datasignal_unsubscribe(signal);

    
    def shutdown_raw(self):
        # stop acc
        libmetawear.mbl_mw_acc_stop(self.device.board)
        libmetawear.mbl_mw_acc_disable_acceleration_sampling(self.device.board)
        
        # stop gyro
        libmetawear.mbl_mw_gyro_bmi160_stop(self.device.board)
        libmetawear.mbl_mw_gyro_bmi160_disable_rotation_sampling(self.device.board)

        # unsubscribe acc
        acc = libmetawear.mbl_mw_acc_get_acceleration_data_signal(self.device.board)
        libmetawear.mbl_mw_datasignal_unsubscribe(acc)
        
        # unsubscribe gyro
        gyro = libmetawear.mbl_mw_gyro_bmi160_get_rotation_data_signal(self.device.board)
        libmetawear.mbl_mw_datasignal_unsubscribe(gyro)
        
        # disconnect
        libmetawear.mbl_mw_debug_disconnect(self.device.board)


    def acc_data_handler (self, ctx, data):
        d = parse_value(data)
        self.acc.append([d.x*9.81, d.y*9.81, (d.z-0.981)*9.81])


    def gyr_data_handler (self, ctx, data):
        d = parse_value(data)
        self.gyr.append([d.x, d.y, d.z])


    def conv_to_lin_acc(self, correct=False):
        # convert acc to lin_acc and return
        if not correct:
            self.lin_acc = np.array(self.acc)
            return
            
        min_len = min(len(self.acc), len(self.gyr))
        self.acc = np.array(self.acc[:min_len])
        self.gyr = np.array(self.gyr[:min_len])
        self.gyr = np.deg2rad(self.gyr)

        # manual sensor fusion to fix orientation
        madgwick = Madgwick()
        Q = np.tile([1., 0., 0., 0.], (len(self.gyr), 1))
        for t in range(1, len(self.gyr)):
            Q[t] = madgwick.updateIMU(Q[t-1], gyr=self.gyr[t], acc=self.acc[t])

        lin_acc = []
        for a, q in zip(self.acc, Q):
            lin_acc.append(Quaternion(q).rotate(a))

        self.lin_acc = np.array(lin_acc)


    def save_lin_acc(self, filename):
        try:
            os.makedirs(".data")
        except OSError:
            pass # data dir already exists

        try:
            with open(os.path.join(".data", filename), 'w') as f:
                writer = csv.writer(f)
                for row in self.lin_acc:
                    writer.writerow([str(r) for r in row])

        except OSError:
            print("Invalid write permissions")


    def calc_vel(self):
        self.vel = scipy.integrate.cumulative_trapezoid(self.lin_acc, 
                axis=0, dx=0.01)


    def calc_pos(self):
        self.pos = scipy.integrate.cumulative_trapezoid(self.vel, 
                axis=0, dx=0.01)


    def plot(self, data, title, units):
        plt.plot(data)
        plt.title = title
        plt.ylabel = units
        plt.show()


if __name__ == "__main__":
    main()

