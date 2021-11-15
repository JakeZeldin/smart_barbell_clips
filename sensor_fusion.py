from mbientlab.metawear import MetaWear, libmetawear, parse_value
from mbientlab.metawear.cbindings import *
import time
import matplotlib.pyplot as plt


def main():

    address = "C7:EA:21:57:F5:E2" #Will
    #address = "C4:A3:A4:75:A2:86" #Matt

    device = MetaWear(address)
    device.connect()

    session = State(device)

    session.startup()
    print("Recording")
    time.sleep(2)
    print("Stopped")
    session.shutdown()

    for q in session.quat_data:
        print(q)

    device.disconnect()


class State():
    def __init__(self, device):
        self.device = device
        self.samples = 0
        self.quat_data = []
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

        # Subscribe to the quaternion signal
        signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(self.device.board, 
                SensorFusionData.QUATERNION);
        libmetawear.mbl_mw_datasignal_subscribe(signal, None, self.callback);

        # Start sensor fusion (acc + gyro + mag + on-board sensor fusion algo)
        libmetawear.mbl_mw_sensor_fusion_enable_data(self.device.board, 
                SensorFusionData.QUATERNION);
        libmetawear.mbl_mw_sensor_fusion_start(self.device.board);

    def shutdown(self):

        # Stop sensor fusion
        libmetawear.mbl_mw_sensor_fusion_stop(self.device.board);

        # Unsubscribe
        signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(self.device.board, 
                SensorFusionData.QUATERNION);
        libmetawear.mbl_mw_datasignal_unsubscribe(signal);


    def data_handler (self, ctx, data):
        d = parse_value(data)
        self.quat_data.append([d.w, d.x, d.y, d.z])
        self.samples += 1


if __name__ == "__main__":
    main()
