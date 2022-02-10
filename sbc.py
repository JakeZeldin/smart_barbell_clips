# from mbientlab.metawear import MetaWear, libmetawear, parse_value
# from mbientlab.metawear.cbindings import *

from random import sample
import time
import matplotlib.pyplot as plt
import numpy as np
import scipy.integrate
import math
import sys
import argparse
import csv
import os
from scipy import signal

# for orienting in global frame
from ahrs.filters import Madgwick
from ahrs import Quaternion

samplePeriod=100 #the sensors operte at 100 Hz

def main():
    
    parser = argparse.ArgumentParser()
    parser.add_argument("-r", nargs='+', 
            help="record with MAC address(es) of sensor(s) (only specify first"
            " two characters) default = C7",
            metavar="MAC")

    parser.add_argument("-s", nargs='+',
            help="save recording or loaded data to .data/CSV",
            metavar="FILESTRING")
    parser.add_argument("-l", nargs='+',
            help="load linear acceleration from .data/CSV",
            metavar="CSV")

    parser.add_argument("-d", action="store_true",
            help="have first sensor enetered perform mbient fusion and second sensor perform wills fusion")
    parser.add_argument("-a", nargs='+',
            choices=['x', 'y', 'z'],
            help="plot acceleration in X, Y, and/or Z axis")
    parser.add_argument("-v", nargs='+',
            choices=['x', 'y', 'z'],
            help="plot velocity in X, Y, and/or Z axis")
    parser.add_argument("-p", nargs='+',
            choices=['x', 'y', 'z', 'yz'],
            help="plot position in X, Y, and/or Z axis")

    parser.add_argument("-f", action="store_true",
            help="use mbient sensor fusion algorithm for absolute orientation "
            "acceleration")
    parser.add_argument("-c", action="store_true",
            help="correct raw acceleration with orientations from "
            "non-proprietary sensor fusion. Has no effect on mbient fusion")
    parser.add_argument("-t", default='5', type=int,
            help="time in T seconds to record for, default 5")
    parser.add_argument("-n", type=int,
            help="number of recordings") 

    # parser.add_argument("-z", "--zupt")
   
    parser.add_argument("-e", type=str, help="after the acceleration has been found, apply an error correction method. Options are butter, new, ...")

    args = parser.parse_args()

    if args.l is None and args.r is None:
        print("Sensors not activated, and no file loaded")
        return
    
    if args.r and len(args.r) != 2 and args.d:
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
                if args.d is True:
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

        #Used for completing multiple data collections at a time 
        if args.n:
            continue_var = args.n
        else:
            continue_var = 1
        
        while(continue_var>=1):

            if args.d is not True:
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
        
        
            if args.d is not True:
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
                        s.save_lin_acc(args.s[i], continue_var)
            else:
                for i,s in enumerate(states):
                    if s.mac == "C4":
                        #if for -d c4 shutdown mbient fusion
                        if C4 == "mfus":
                            s.shutdown_fusion()
                            s.conv_to_lin_acc()
                        #if for -d c4 shutdown raw data collection
                        else:
                            s.shutdown_raw()
                            s.conv_to_lin_acc(correct=True)
                    
                        if args.s is not None:
                            s.save_lin_acc(args.s[i], continue_var)

                    if s.mac =="C7":
                        #if for -d c7 shutdown mbient fusion
                        if C7 == "mfus":
                            s.shutdown_fusion()
                            s.conv_to_lin_acc()
                        #if for -d c4 shutdown raw data collection 
                        else:
                            s.shutdown_raw()
                            s.conv_to_lin_acc(correct=True)

                        if args.s is not None:
                            s.save_lin_acc(args.s[i], continue_var)

            if not args.n:
                redo = 1
                while (redo):
                    user_input = input("Enter Y to rerun code again and N to disconnect")
            
                    if (user_input == "Y"):
                        continue_var= continue_var + 1
                        redo = 0
                    elif (user_input == "N"):
                        continue_var = 0
                        redo = 0
                    else:
                        print("Unknown user entry redo")
                        redo = 1
            else:
                continue_var -= 1
        for s in states:
            s.disconnect_sensor()
            
    if args.l is not None:
            for file_name in args.l:
                states.append(State(None, file_name))

    #data is now found as acceleration  
    for s in states:
        # print("\naccel before error correction:")
        # print(s.lin_acc)
        # print(len(s.lin_acc))
        if args.e is not None:
            s.method = args.e
            print("Method selected for error correction: ",s.method)
            # error correction methods will take in acceleration, then store acc, vel, and pos
            # in s.accCorrected, s.velCorrected, s.posCorrected, respectively 
            s.calc_vel()
            s.calc_pos()
            s.error_correction(s.method)

        else:
            s.calc_vel()
            s.calc_pos()

        #printing time
        np.set_printoptions(suppress=True, precision=3)
        
        # print("acceleration:")
        # print(len(s.lin_acc))
        # for i in range(len(s.lin_acc)):
        #     print(s.lin_acc[i])
        #     print(s.accCorrected[i])
        #     print("\n")
        
        # print("\n\nvelocity:")
        # print(len(s.vel))
        # for i in range(len(s.vel)):
        #     print(s.vel[i])
        #     print(s.velCorrected[i])
        #     print("\n")
        
        # print("\n\nposition:")
        # print(len(s.pos))
        # for i in range(len(s.pos)):
        #     print(s.pos[i])
        #     print(s.posCorrected[i])
        #     print("\n")
        
        if args.e is not  None:
            s.lin_acc = s.accCorrected
            s.vel = s.velCorrected
            s.pos_acc = s.posCorrected
        
        if args.a is not None:
            for axis in args.a:
                if axis == 'x':
                    s.plot(s.lin_acc[:,0], f'''X Acceleration - {s.filename}''', 'meters/second^2')
                if axis == 'y':
                    s.plot(s.lin_acc[:,1], f'''Y Acceleration - {s.filename}''', 'meters/second')
                if axis == 'z':
                    s.plot(s.lin_acc[:,2], f'''Z Acceleration - {s.filename}''', 'meters')	
        if args.v is not None:
            for axis in args.v:
                if axis == 'x':
                    s.plot(s.vel[:,0], f'''X Velocity - {s.filename}''', 'NOT SURE')
                if axis == 'y':
                    s.plot(s.vel[:,1], f'''Y Velocity - {s.filename}''', 'NOT SURE')
                if axis == 'z':
                    s.plot(s.vel[:,2], f'''Z Velocity - {s.filename}''', 'NOT SURE')
        if args.p is not None:
            for axis in args.p:
                if axis == 'x':
                    s.plot(s.pos[:,0], f'''X Postion - {s.filename}''', 'NOT SURE')
                if axis == 'y':
                    s.plot(s.pos[:,1], f'''Y Postion - {s.filename}''', 'NOT SURE')
                if axis == 'z':
                    s.plot(s.pos[:,2], f'''Z Postion - {s.filename}''', 'NOT SURE')
                if axis == 'yz':
                    s.plot2D(s.pos[:,1], s.pos[:,2], f'''Y Z Postion - {s.filename} - {s.method}''', 'NOT SURE')


class State():
    def __init__(self, device, filename=None, mac=None):
        self.device = device
        self.acc = []
        self.gyr = []
        self.mac = mac
        self.filename = filename
        self.method = "NoCorrection"

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

        self.vel = []
        self.pos = []

        self.accCorrected = []
        self.velCorrected = []
        self.posCorrected = []

        # self.acc_callback = FnVoid_VoidP_DataP(self.acc_data_handler)
        # self.gyr_callback = FnVoid_VoidP_DataP(self.gyr_data_handler)

        self.signal_acc = None
        self.signal_gyr = None
        self.signal_fus = None


    # def start_fusion(self):

    #     # Sensor fusion setup
    #     libmetawear.mbl_mw_sensor_fusion_set_mode(self.device.board, 
    #             SensorFusionMode.IMU_PLUS);
    #     libmetawear.mbl_mw_sensor_fusion_set_acc_range(self.device.board, 
    #             SensorFusionAccRange._8G);
    #     libmetawear.mbl_mw_sensor_fusion_set_gyro_range(self.device.board, 
    #             SensorFusionGyroRange._2000DPS);
    #     libmetawear.mbl_mw_sensor_fusion_write_config(self.device.board);

    #     # Subscribe to the linear acceleration signal
    #     signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(self.device.board, 
    #             SensorFusionData.CORRECTED_ACC);
    #     libmetawear.mbl_mw_datasignal_subscribe(signal, None, self.acc_callback);

    #     # Start sensor fusion (acc + gyro + on-board sensor fusion algo)
    #     libmetawear.mbl_mw_sensor_fusion_enable_data(self.device.board, 
    #             SensorFusionData.CORRECTED_ACC);
    #     libmetawear.mbl_mw_sensor_fusion_start(self.device.board);


    # def start_raw(self):

    #     libmetawear.mbl_mw_acc_bmi160_set_odr(self.device.board,
    #             AccBmi160Odr._100Hz) # BMI 160 specific call
    #     libmetawear.mbl_mw_acc_bosch_set_range(self.device.board, AccBoschRange._4G)
    #     libmetawear.mbl_mw_acc_write_acceleration_config(self.device.board)

    #     # config gyro
    #     libmetawear.mbl_mw_gyro_bmi160_set_range(self.device.board, GyroBoschRange._1000dps);
    #     libmetawear.mbl_mw_gyro_bmi160_set_odr(self.device.board,
    #             GyroBoschOdr._100Hz);
    #     libmetawear.mbl_mw_gyro_bmi160_write_config(self.device.board);

    #     # get acc signal and subscribe
    #     acc = libmetawear.mbl_mw_acc_get_acceleration_data_signal(self.device.board)
    #     libmetawear.mbl_mw_datasignal_subscribe(acc, None, self.acc_callback)

    #     # get gyro signal and subscribe
    #     gyro = libmetawear.mbl_mw_gyro_bmi160_get_rotation_data_signal(self.device.board)
    #     libmetawear.mbl_mw_datasignal_subscribe(gyro, None, self.gyr_callback)

    #     # start acc
    #     libmetawear.mbl_mw_acc_enable_acceleration_sampling(self.device.board)
    #     libmetawear.mbl_mw_acc_start(self.device.board)

    #     # start gyro
    #     libmetawear.mbl_mw_gyro_bmi160_enable_rotation_sampling(self.device.board)
    #     libmetawear.mbl_mw_gyro_bmi160_start(self.device.board)


    # def shutdown_fusion(self):
    #     # Stop sensor fusion
    #     libmetawear.mbl_mw_sensor_fusion_stop(self.device.board);

    #     # Unsubscribe
    #     signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(self.device.board, 
    #             SensorFusionData.CORRECTED_ACC);
    #     libmetawear.mbl_mw_datasignal_unsubscribe(signal);

    
    # def shutdown_raw(self):
    #     # stop acc
    #     libmetawear.mbl_mw_acc_stop(self.device.board)
    #     libmetawear.mbl_mw_acc_disable_acceleration_sampling(self.device.board)
        
    #     # stop gyro
    #     libmetawear.mbl_mw_gyro_bmi160_stop(self.device.board)
    #     libmetawear.mbl_mw_gyro_bmi160_disable_rotation_sampling(self.device.board)

    #     # unsubscribe acc
    #     acc = libmetawear.mbl_mw_acc_get_acceleration_data_signal(self.device.board)
    #     libmetawear.mbl_mw_datasignal_unsubscribe(acc)
        
    #     # unsubscribe gyro
    #     gyro = libmetawear.mbl_mw_gyro_bmi160_get_rotation_data_signal(self.device.board)
    #     libmetawear.mbl_mw_datasignal_unsubscribe(gyro)
        

    def disconnect_sensor(self):
        libmetawear.mbl_mw_debug_disconnect(self.device.board)

    # def acc_data_handler (self, ctx, data):
    #     d = parse_value(data)
    #     self.acc.append([d.x*9.81, d.y*9.81, (d.z-0.981)*9.81])


    # def gyr_data_handler (self, ctx, data):
    #     d = parse_value(data)
    #     self.gyr.append([d.x, d.y, d.z])


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


    def save_lin_acc(self, filename, file_number):
        try:
            os.makedirs(".data")
        except OSError:
            pass # data dir already exists

        try:
            with open(os.path.join(".data", filename + str(file_number) + ".csv"), 'w') as f:
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


    def error_correction(self, method):
        # this is where new error correction methods are implemented    
         if method == "new":
            print("new error correction method")
            return
         elif method == "butter": 
            #split up acc in to x,y,z
            #these were all multiplied by 9.81, but we dont need to do that anymore?
            acc_x = [(acc[0]) for acc in self.lin_acc]
            acc_y = [(acc[1]) for acc in self.lin_acc]
            acc_z = [(acc[2]) for acc in self.lin_acc]

            # start of some funky acceleration stuff
            # initialize empty or 0 arrays
            acc_mag = [None]*len(acc_z)
            acc_magFilt = [None]*len(acc_z)
            stationary = [0]*len(acc_z)

            
            # acceleraton magnitude --> square root of (x^2 + y^2 + z^2)
            for i in range(len(acc_z)):
                acc_mag[i] =math.sqrt(acc_x[i]*acc_x[i]+acc_y[i]*acc_y[i]+acc_z[i]*acc_z[i]) 
            
            
            filterCutOff1 = 0.001
            b,a = signal.butter(1,(2*filterCutOff1)/samplePeriod,'high')
            acc_magFilt = signal.filtfilt(b , a, acc_mag)
            
            for i in range(len(acc_magFilt)):
                acc_magFilt[i] = abs(acc_magFilt[i])


            filterCutOff2 = 5
            b,a = signal.butter(1,(2*filterCutOff2)/samplePeriod,'low')
            acc_magFilt= signal.filtfilt(b , a, acc_magFilt)
            
            for i in range(len(acc_magFilt)):
                acc_magFilt[i] = abs(acc_magFilt[i])
                
                
            stationaryThresh=0.05
            if acc_magFilt[i] < stationaryThresh:
                stationary[i] = 1
            
            
            # change acceleration value to 0 if it is below a threshold
            xAccThresh = 0.02
            yAccThresh = 0.02
            zAccThresh = 0.02
            for i in range(len(acc_z)):
                if (abs(acc_x[i]) < xAccThresh):
                    acc_x[i] = 0.00
                if (abs(acc_y[i]) < yAccThresh):
                    acc_y[i] = 0.00
                if (abs(acc_z[i]) < zAccThresh):
                    acc_z[i] = 0.00

            for i in range(len(acc_z)):
                self.accCorrected.append([acc_x[i],acc_y[i],acc_z[i]])

            self.accCorrected = np.array(self.accCorrected)
            #acceleration done, now print it
            # print("\n\n\nacceleration: ")
            # for i in range(len(acc_z)): 
            #     print(float("{:.2f}".format(acc_x[i])),float("{:.2f}".format(acc_y[i])),float("{:.2f}".format(acc_x[i])))
            # print("\n accel post butter filtration")
            # for i in range(len(acc_z)):
            #     print("\n")
            #     print(self.lin_acc[i][2])
            #     print(acc_z[i])
            
            # velocity begins
            vel_x = [0] 
            vel_y = [0] 
            vel_z = [0] 
            
            for i in range (1, len(acc_x) - 1):
                    vel_x.append(vel_x[i-1] + acc_x[i]*(1/samplePeriod))
                    vel_y.append(vel_y[i-1] + acc_y[i]*(1/samplePeriod))
                    vel_z.append(vel_z[i-1] + acc_z[i]*(1/samplePeriod))
                    if stationary[i] == 1:
                        vel_x[i] = 0
                        vel_y[i] = 0
                        vel_z[i] = 0
            
            vel_xBasic = scipy.integrate.cumulative_trapezoid(acc_x)
            vel_yBasic = scipy.integrate.cumulative_trapezoid(acc_y)
            vel_zBasic = scipy.integrate.cumulative_trapezoid(acc_z)
            

            # for i in range(len(acc_z)-1):
            #     print("\n")
            #     print(vel_z[i])
            #     print(vel_zBasic[i])

            
            velDrift_x = [0]*len(vel_x)
            velDrift_y = [0]*len(vel_y)
            velDrift_z = [0]*len(vel_z)
            stationary_start = [0]*len(stationary)
            staionary_end    = [0]*len(stationary)


            
            stationary_start = np.argwhere(np.diff(stationary)== -1)
            stationary_end   = np.argwhere(np.diff(stationary)== 1)
            
            vel_drift_x = [0]*len(vel_x)
            vel_drift_y = [0]*len(vel_y)
            vel_drift_z = [0]*len(vel_z)
            for i in range(stationary_end.size):
                driftRate_x = (vel_x[stationary_end[i,0]-1])/(stationary_end[i,0]- stationary_start[i,0])
                driftRate_y = (vel_y[stationary_end[i,0]-1])/(stationary_end[i,0]- stationary_start[i,0])
                driftRate_z = (vel_z[stationary_end[i,0]-1])/(stationary_end[i,0]- stationary_start[i,0])
                
                
                enum = [0]
                range_var = stationary_end[i,0]-stationary_start[i,0]
                #print("range")
                #print(range_var)
                drift_var_x = [None]*range_var
                drift_var_y = [None]*range_var
                drift_var_z = [None]*range_var

            
                for w in range(0,range_var):
                    drift_var_x[w] = w*driftRate_x
                    drift_var_y[w] = w*driftRate_y
                    drift_var_z[w] = w*driftRate_z
                
                for w in range(range_var-1):
                    
                    start = w+stationary_start[0,0]
                    
                    vel_drift_x[w+stationary_start[0,0]] = drift_var_x[w]
                    vel_drift_y[w+stationary_start[0,0]] = drift_var_y[w]
                    vel_drift_z[w+stationary_start[0,0]] = drift_var_z[w]

        

            for i in range(len(vel_x)):
                vel_x[i] = vel_x[i] - vel_drift_x[i]
                vel_y[i] = vel_y[i] - vel_drift_y[i]
                vel_z[i] = vel_z[i] - vel_drift_z[i]
            

            for i in range(len(vel_z)):
                self.velCorrected.append([vel_x[i],vel_y[i],vel_z[i]])
            
            self.velCorrected = np.array(self.velCorrected)
            #velocity done, now print it
            # print("\n\n\nvelocity: ")
            # for i in range(len(vel_z)): 
            #     print(float("{:.2f}".format(vel_x[i])),float("{:.2f}".format(vel_y[i])),float("{:.2f}".format(vel_z[i])))
            
        
        
            #position begins
            pos_x = [0] 
            pos_y = [0] 
            pos_z = [0]


            
            for i in range (1, len(vel_x) - 1):
                pos_x.append(pos_x[i-1] + vel_x[i]*0.01)
                pos_y.append(pos_y[i-1] + vel_y[i]*0.01)
                pos_z.append(pos_z[i-1] + vel_z[i]*0.01)

            
            pos_xBasic = scipy.integrate.cumulative_trapezoid(vel_x)
            pos_yBasic = scipy.integrate.cumulative_trapezoid(vel_y)
            pos_zBasic = scipy.integrate.cumulative_trapezoid(vel_z)
            
            for i in range(len(pos_z)):
                self.posCorrected.append([pos_x[i],pos_y[i],pos_z[i]])
            
            self.posCorrected = np.array(self.posCorrected)

            
            return
         else:
            print("invalid method for error correction")
            return

    def plot(self, data, title, units):
        if not os.path.exists(os.path.join(os.getcwd(),f'plots/{self.filename}')):
            os.mkdir(f'plots/{self.filename}')
        print(f'Saving {title}')
        plt.plot(data)
        plt.title(title)
        plt.ylabel(units)
        plt.savefig(f'plots/{self.filename}/{title}.png')
        #plt.show() #Optional to show plot
        plt.clf()

    def plot2D(self, dataX, dataY, title, units):
        if not os.path.exists(os.path.join(os.getcwd(),f'plots/{self.filename}')):
            os.mkdir(f'plots/{self.filename}')
        print(f'Saving {title}')
        plt.plot(dataX, dataY)
        plt.title(title)
        plt.ylabel(units)
        plt.savefig(f'plots/{self.filename}/{title}.png')
        #plt.show() #Optional to show plot
        plt.clf()
if __name__ == "__main__":
    main()

