from mbientlab.metawear import MetaWear, libmetawear, parse_value
from mbientlab.metawear.cbindings import *
import time
import matplotlib.pyplot as plt
import scipy.integrate
from mpl_toolkits.mplot3d import Axes3D


def main():

    #address = "C7:EA:21:57:F5:E2" #Will
    address = "C4:A3:A4:75:A2:86" #Matt

    device = MetaWear(address)
    device.connect()

    session = State(device)

    session.startup()
    print("Recording")
    time.sleep(5)
    print("Stopped")
    session.shutdown()

    acc_x = [acc[0]*9.81 for acc in session.lin_acc]
    acc_y = [acc[1]*9.81 for acc in session.lin_acc]
    acc_z = [(acc[2]*9.81) for acc in session.lin_acc]

    for i in range(len(acc_z)):
        if (abs(acc_x[i]) < 0.2):
            acc_x[i] = 0
        if (abs(acc_y[i]) < 0.2):
            acc_y[i] = 0
        if (abs(acc_z[i]) < 0.2):
            acc_z[i] = 0
        print(acc_x[i], acc_y[i], acc_z[i])


    vel_x = [0] 
    vel_y = [0] 
    vel_z = [0] 
    for i in range (1, len(acc_x) - 1):
        vel_x.append(vel_x[i-1] + acc_x[i]*0.01)
        vel_y.append(vel_y[i-1] + acc_y[i]*0.01)
        vel_z.append(vel_z[i-1] + acc_z[i]*0.01)

    # vel_x = scipy.integrate.cumulative_trapezoid(acc_x)
    # vel_y = scipy.integrate.cumulative_trapezoid(acc_y)
    # vel_z = scipy.integrate.cumulative_trapezoid(acc_z)

    pos_x = [0] 
    pos_y = [0] 
    pos_z = [0] 
    for i in range (1, len(vel_x) - 1):
        pos_x.append(pos_x[i-1] + vel_x[i]*0.01)
        pos_y.append(pos_y[i-1] + vel_y[i]*0.01)
        pos_z.append(pos_z[i-1] + vel_z[i]*0.01)

    # pos_x = scipy.integrate.cumulative_trapezoid(vel_x)
    # pos_y = scipy.integrate.cumulative_trapezoid(vel_y)
    # pos_z = scipy.integrate.cumulative_trapezoid(vel_z)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim3d(min([min(pos_x), min(pos_y), min(pos_z)]), max([max(pos_x),
        max(pos_y), max(pos_z)]))
    ax.set_ylim3d(min([min(pos_x), min(pos_y), min(pos_z)]), max([max(pos_x),
        max(pos_y), max(pos_z)]))
    ax.set_zlim3d(min([min(pos_x), min(pos_y), min(pos_z)]), max([max(pos_x),
        max(pos_y), max(pos_z)]))
    ax.plot(pos_x, pos_y, pos_z)
    plt.show()
    Axes3D.plot()

    device.on_disconnect = lambda status: print("disconnected")
    device.disconnect()


class State():
    def __init__(self, device):
        self.device = device
        self.samples = 0
        self.lin_acc = []
        self.callback = FnVoid_VoidP_DataP(self.data_handler)

    def startup(self):

        # # Setup the accelerometer sample frequency and range
        # libmetawear.mbl_mw_acc_set_odr(self.device.board, 100.0)
        # libmetawear.mbl_mw_acc_set_range(self.device.board, 16.0)
        # libmetawear.mbl_mw_acc_write_acceleration_config(self.device.board)

        # # Get the accelerometer data signal
        # signal = libmetawear.mbl_mw_acc_get_acceleration_data_signal(self.device.board)
        # # Subscribe to it
        # libmetawear.mbl_mw_datasignal_subscribe(signal, None, self.callback)

        # # Enable the accelerometer
        # libmetawear.mbl_mw_acc_enable_acceleration_sampling(self.device.board)
        # libmetawear.mbl_mw_acc_start(self.device.board)

        # Sensor fusion setup
        libmetawear.mbl_mw_sensor_fusion_set_mode(self.device.board, 
                SensorFusionMode.IMU_PLUS);
        libmetawear.mbl_mw_sensor_fusion_set_acc_range(self.device.board, 
                SensorFusionAccRange._8G);
        libmetawear.mbl_mw_sensor_fusion_set_gyro_range(self.device.board, 
                SensorFusionGyroRange._2000DPS);
        libmetawear.mbl_mw_sensor_fusion_write_config(self.device.board);

        # Subscribe to the quaternion signal
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
        self.lin_acc.append([d.x, d.y, d.z])

        self.samples += 1


if __name__ == "__main__":
    main()
