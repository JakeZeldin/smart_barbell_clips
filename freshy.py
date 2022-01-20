from mbientlab.metawear import MetaWear, libmetawear, parse_value, cbindings
from mbientlab.metawear.cbindings import *
import time
import math
import matplotlib.pyplot as plt
import scipy.integrate
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from threading import Event


def main():
    np.set_printoptions(precision=3, suppress=True)

    address = "C7:EA:21:57:F5:E2" #Will
    #address = "C4:A3:A4:75:A2:86" #Matt

    device = MetaWear(address)
    device.connect()

    session = Raw(device)

    session.startup()
    print("Recording")
    time.sleep(5)
    print("Stopped")
    session.shutdown()
    
    min_len = min(len(session.gyr_x), len(session.acc_x))
    acc_x = np.array(session.acc_x[:min_len])
    acc_y = np.array(session.acc_y[:min_len])
    acc_z = np.array(session.acc_z[:min_len])

    acc_x = np.reshape(acc_x, (len(acc_x), 1))
    acc_y = np.reshape(acc_y, (len(acc_y), 1))
    acc_z = np.reshape(acc_z, (len(acc_z), 1))

    gyr_x = np.array(session.gyr_x[:min_len])
    gyr_y = np.array(session.gyr_y[:min_len])
    gyr_z = np.array(session.gyr_z[:min_len])

    gyr_x = np.reshape(gyr_x, (len(gyr_x), 1))
    gyr_y = np.reshape(gyr_y, (len(gyr_y), 1))
    gyr_z = np.reshape(gyr_z, (len(gyr_z), 1))

    acc = np.concatenate((acc_x, acc_y, acc_z), axis = 1)
    gyr = np.concatenate((gyr_x, gyr_y, gyr_z), axis = 1)
    print(gyr[0])
    gyr = np.deg2rad(gyr)
    print(gyr[0])

    from ahrs.filters import Madgwick
    from ahrs import Quaternion

    madgwick = Madgwick()
    Q = np.tile([1., 0., 0., 0.], (len(gyr), 1)) # Allocate for quaternions
    for t in range(1, len(gyr)):
        Q[t] = madgwick.updateIMU(Q[t-1], gyr=gyr[t], acc=acc[t])

    lin_acc = []
    for a, q in zip(acc, Q):
        lin_acc.append(Quaternion(q).rotate(a))

    lin_acc = np.array(lin_acc)

    print(lin_acc)

        



    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.set_xlim3d(min([min(pos_x), min(pos_y), min(pos_z)]), max([max(pos_x),
    #     max(pos_y), max(pos_z)]))
    # ax.set_ylim3d(min([min(pos_x), min(pos_y), min(pos_z)]), max([max(pos_x),
    #     max(pos_y), max(pos_z)]))
    # ax.set_zlim3d(min([min(pos_x), min(pos_y), min(pos_z)]), max([max(pos_x),
    #     max(pos_y), max(pos_z)]))
    # ax.plot(pos_x, pos_y, pos_z)
    # plt.show()
    # Axes3D.plot()

class State():
    def __init__(self, device):
        self.device = device
        self.samples = 0
        self.acc_data = []
        self.callback = FnVoid_VoidP_DataP(self.data_handler)

    def startup(self):
        # Sensor fusion setup
        libmetawear.mbl_mw_sensor_fusion_set_mode(self.device.board, 
                SensorFusionMode.NDOF);
        libmetawear.mbl_mw_sensor_fusion_set_acc_range(self.device.board, 
                SensorFusionAccRange._8G);
        libmetawear.mbl_mw_sensor_fusion_set_gyro_range(self.device.board, 
                SensorFusionGyroRange._2000DPS);
        libmetawear.mbl_mw_sensor_fusion_write_config(self.device.board);

        # Subscribe to the linear acceleration signal
        signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(self.device.board, 
                SensorFusionData.LINEAR_ACC);
        libmetawear.mbl_mw_datasignal_subscribe(signal, None, self.callback);

        # Start sensor fusion (acc + gyro + mag + on-board sensor fusion algo)
        libmetawear.mbl_mw_sensor_fusion_enable_data(self.device.board, 
                SensorFusionData.LINEAR_ACC);
        libmetawear.mbl_mw_sensor_fusion_start(self.device.board);

    def shutdown(self):

        # Stop sensor fusion
        libmetawear.mbl_mw_sensor_fusion_stop(self.device.board);

        # Unsubscribe
        signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(self.device.board, 
                SensorFusionData.LINEAR_ACC);
        libmetawear.mbl_mw_datasignal_unsubscribe(signal);


    def data_handler (self, ctx, data):
        d = parse_value(data)
        self.acc_data.append([d.x, d.y, d.z])
        self.samples += 1


class Raw():
    def __init__(self, device):
        self.device = device
        self.acc_samples = 0
        self.gyr_samples = 0
        self.acc_x = []
        self.acc_y = []
        self.acc_z = []
        self.acc_epochs = []

        self.gyr_x = []
        self.gyr_y = []
        self.gyr_z = []
        self.gyr_epochs = []

        self.acc_callback = FnVoid_VoidP_DataP(self.acc_data_handler)
        self.gyr_callback = FnVoid_VoidP_DataP(self.gyr_data_handler)
        self.signal_acc = 0
        self.signal_gyr = 0
    
    def startup(self):

        #libmetawear.mbl_mw_acc_set_odr(self.device.board, 50.0) # Generic call
        libmetawear.mbl_mw_acc_bmi160_set_odr(self.device.board,
                AccBmi160Odr._50Hz) # BMI 160 specific call
        libmetawear.mbl_mw_acc_bosch_set_range(self.device.board, AccBoschRange._4G)
        libmetawear.mbl_mw_acc_write_acceleration_config(self.device.board)

        # config gyro
        libmetawear.mbl_mw_gyro_bmi160_set_range(self.device.board, GyroBoschRange._1000dps);
        libmetawear.mbl_mw_gyro_bmi160_set_odr(self.device.board,
                GyroBoschOdr._50Hz);
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

    def shutdown(self):
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
        self.acc_x.append(d.x)
        self.acc_y.append(d.y)
        self.acc_z.append(d.z)
        self.acc_samples += 1
        self.acc_epochs.append(data.contents.epoch)

    def gyr_data_handler (self, ctx, data):
        d = parse_value(data)
        self.gyr_x.append(d.x)
        self.gyr_y.append(d.y)
        self.gyr_z.append(d.z)
        self.gyr_samples += 1
        self.gyr_epochs.append(data.contents.epoch)

class DataFusion:
    # init
    def __init__(self, device):
        self.device = device
        self.callback = cbindings.FnVoid_VoidP_DataP(self.data_handler)
        self.processor = None

    # download data callback fxn
    def data_handler(self, ctx, data):
        values = parse_value(data, n_elem = 2)
        print("acc: (%.4f,%.4f,%.4f), gyro; (%.4f,%.4f,%.4f)" % (values[0].x, values[0].y, values[0].z, values[1].x, values[1].y, values[1].z))

    def setup(self):
        # ble settings
        libmetawear.mbl_mw_settings_set_connection_parameters(self.device.board, 7.5, 7.5, 0, 6000)
        time.sleep(1.5)
        # events
        e = Event()
        # processor callback fxn
        def processor_created(context, pointer):
            self.processor = pointer
            e.set()
        # processor fxn ptr
        fn_wrapper = cbindings.FnVoid_VoidP_VoidP(processor_created)
        # get acc signal
        acc = libmetawear.mbl_mw_acc_get_acceleration_data_signal(self.device.board)
        # get gyro signal - MMRl, MMR, MMc ONLY
        gyro = libmetawear.mbl_mw_gyro_bmi160_get_rotation_data_signal(self.device.board)
        # get gyro signal - MMRS ONLY
        #gyro = libmetawear.mbl_mw_gyro_bmi270_get_rotation_data_signal(self.device.board)
        # create signals variable
        signals = (c_void_p * 1)()
        signals[0] = gyro
        # create acc + gyro signal fuser
        libmetawear.mbl_mw_dataprocessor_fuser_create(acc, signals, 1, None, fn_wrapper)
        # wait for fuser to be created
        e.wait()
        # subscribe to the fused signal
        libmetawear.mbl_mw_datasignal_subscribe(self.processor, None, self.callback)

    def start(self):
        # start gyro sampling - MMRL, MMC, MMR only
        libmetawear.mbl_mw_gyro_bmi160_enable_rotation_sampling(self.device.board)

        # start gyro sampling - MMS ONLY
        #libmetawear.mbl_mw_gyro_bmi270_enable_rotation_sampling(self.device.board)

        # start acc sampling
        libmetawear.mbl_mw_acc_enable_acceleration_sampling(self.device.board)
        # start gyro - MMRL, MMC, MMR only
        libmetawear.mbl_mw_gyro_bmi160_start(self.device.board)
        # start gyro sampling - MMS ONLY
        #libmetawear.mbl_mw_gyro_bmi270_start(self.device.board)
        # start acc
        libmetawear.mbl_mw_acc_start(self.device.board)

        
if __name__ == "__main__":
    main()
