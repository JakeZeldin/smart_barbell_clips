#edit
from mbientlab.metawear import MetaWear, libmetawear, parse_value
from mbientlab.metawear.cbindings import *
import time
import matplotlib.pyplot as plt


def main():

    address = "C7:EA:21:57:F5:E2" #Will
    #address = "C4:A3:A4:75:A2:86" #Matt
    device = MetaWear(address)
    device.connect()

    pattern = LedPattern(repeat_count=Const.LED_REPEAT_INDEFINITELY)
    libmetawear.mbl_mw_led_load_preset_pattern(byref(pattern), LedPreset.SOLID)
    libmetawear.mbl_mw_led_write_pattern(device.board, 
         byref(pattern), LedColor.GREEN)
    libmetawear.mbl_mw_led_play(device.board)

    time.sleep(5.0)
    libmetawear.mbl_mw_led_stop_and_clear(device.board)
    time.sleep(1.0)

    matt = test(device)

    device.disconnect()

class test():

    x_acc = []
    y_acc = []
    z_acc = []

    x_gyr = []
    y_gyr = []
    z_gyr = []

    start_time = 0

    time_acc = []
    time_gyr = []

    def __init__(self,device):

        self.device = device
        # Callback function pointer
        callback_acc = FnVoid_VoidP_DataP(self.data_handler_acc)
        callback_gyr = FnVoid_VoidP_DataP(self.data_handler_gyr)
        callback     = FnVoid_VoidP_DataP(self.data_handler)

        # Setup the accelerometer sample frequency and range
        libmetawear.mbl_mw_acc_set_odr(device.board, 100.0)
        libmetawear.mbl_mw_acc_set_range(device.board, 16.0)
        libmetawear.mbl_mw_acc_write_acceleration_config(device.board)

        # Get the accelerometer data signal
        signal = libmetawear.mbl_mw_acc_get_acceleration_data_signal(device.board)
        # Subscribe to it
        libmetawear.mbl_mw_datasignal_subscribe(signal, None, callback_acc)

        # Enable the accelerometer
        libmetawear.mbl_mw_acc_enable_acceleration_sampling(device.board)
        libmetawear.mbl_mw_acc_start(device.board)

        # Get the gyroscope data signal
        signal = libmetawear.mbl_mw_gyro_bmi270_get_packed_rotation_data_signal(device.board)
        # Subscribe to it
        libmetawear.mbl_mw_datasignal_subscribe(signal, None, callback_gyr)

        # Enable the gyroscope
        libmetawear.mbl_mw_gyro_bmi270_enable_rotation_sampling(device.board)
        libmetawear.mbl_mw_gyro_bmi270_start(device.board)

        # Sensor fusion setup
        libmetawear.mbl_mw_sensor_fusion_set_mode(device.board, 
                SensorFusionMode.IMU_PLUS);
        libmetawear.mbl_mw_sensor_fusion_set_acc_range(device.board, 
                SensorFusionAccRange._8G);
        libmetawear.mbl_mw_sensor_fusion_set_gyro_range(device.board, 
                SensorFusionGyroRange._2000DPS);
        libmetawear.mbl_mw_sensor_fusion_write_config(device.board);

        # Subscribe to the quaternion signal
        signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(device.board, 
                SensorFusionData.QUATERNION);
        libmetawear.mbl_mw_datasignal_subscribe(signal, None, callback);


        # Start sensor fusion (acc + gyro + mag + on-board sensor fusion algo)
        libmetawear.mbl_mw_sensor_fusion_enable_data(device.board, 
                SensorFusionData.QUATERNION);
        libmetawear.mbl_mw_sensor_fusion_start(device.board);

        # Start time to track data
        
        start_time = time.time()       


        print("Tracking data has begun")

        time.sleep(30.0)

        print("Tracking data has stopped")

        # Stop sensor fusion
        libmetawear.mbl_mw_sensor_fusion_stop(device.board);

        # Unsubscribe
        signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(device.board, 
                SensorFusionData.QUATERNION);
        libmetawear.mbl_mw_datasignal_unsubscribe(signal);

        # Disable the gyroscope
        libmetawear.mbl_mw_gyro_bmi160_stop(device.board)
        libmetawear.mbl_mw_gyro_bmi160_disable_rotation_sampling(device.board)

        # Unsubscribe to it
        libmetawear.mbl_mw_datasignal_unsubscribe(signal)
        libmetawear.mbl_mw_debug_disconnect(device.board)


        # Disable the accelerometer
        libmetawear.mbl_mw_acc_stop(device.board)
        libmetawear.mbl_mw_acc_disable_acceleration_sampling(device.board)

        # Unsubscribe to it
        signal = libmetawear.mbl_mw_acc_get_acceleration_data_signal(device.board)
        libmetawear.mbl_mw_datasignal_unsubscribe(signal)
        libmetawear.mbl_mw_debug_disconnect(device.board)

        figs, axs = plt.subplots(1,2)
        axs[0].plot(self.time_acc,self.x_acc, label = "X")
        axs[0].plot(self.time_acc,self.y_acc, label = "Y")
        axs[0].plot(self.time_acc,self.z_acc, label = "Z")
        axs[0].set_title('Acc')
        axs[0].legend()

        axs[1].plot(self.time_gyr,self.x_gyr, label = "X")
        axs[1].plot(self.time_gyr,self.y_gyr, label = "Y")
        axs[1].plot(self.time_gyr,self.z_gyr, label = "Z")
        axs[1].set_title('Gyr')
        axs[1].legend()

        plt.show()


    # Callback function to process/parse the gyroscope data
    def data_handler_acc(self, ctx, data):
        #print("%s    : %s -> %s" % ("Accelrometer" \
        #    self.device.address, parse_value(data)))
        self. x_acc.append(parse_value(data).x)
        self.y_acc.append(parse_value(data).y)
        self.z_acc.append(parse_value(data).z)
        self.time_acc.append(time.time()-self.start_time)

    def data_handler_gyr(self, ctx, data):
        #print("%s    : %s -> %s" % ("Gyroscope", \
        #    self.device.address, parse_value(data)))
        self. x_gyr.append(parse_value(data).x)
        self.y_gyr.append(parse_value(data).y)
        self.z_gyr.append(parse_value(data).z)
        self.time_gyr.append(time.time()-self.start_time)
    
    def data_handler (self, ctx, data):
        print("%s    : %s -> %s" % ("New", self.device.address, parse_value(data)))


if __name__ == "__main__":
    main()
