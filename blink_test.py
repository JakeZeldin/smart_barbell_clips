from mbientlab.metawear import MetaWear, libmetawear
from mbientlab.metawear.cbindings import *
import time
import matplotlib.pyplot as plt

_value_parsers = {
    DataTypeId.UINT32: lambda p: cast(p.contents.value, POINTER(c_uint)).contents.value,
    DataTypeId.INT32: lambda p: cast(p.contents.value, POINTER(c_int)).contents.value,
    DataTypeId.FLOAT: lambda p: cast(p.contents.value, POINTER(c_float)).contents.value,
    DataTypeId.CARTESIAN_FLOAT: lambda p: cast(p.contents.value, POINTER(CartesianFloat)).contents,
    DataTypeId.BATTERY_STATE: lambda p: cast(p.contents.value, POINTER(BatteryState)).contents,
    DataTypeId.TCS34725_ADC: lambda p: cast(p.contents.value, POINTER(Tcs34725ColorAdc)).contents,
    DataTypeId.EULER_ANGLE: lambda p: cast(p.contents.value, POINTER(EulerAngles)).contents,
    DataTypeId.QUATERNION: lambda p: cast(p.contents.value, POINTER(Quaternion)).contents,
    DataTypeId.CORRECTED_CARTESIAN_FLOAT: lambda p: cast(p.contents.value, POINTER(CorrectedCartesianFloat)).contents,
    DataTypeId.OVERFLOW_STATE: lambda p: cast(p.contents.value, POINTER(OverflowState)).contents,
    DataTypeId.LOGGING_TIME: lambda p: cast(p.contents.value, POINTER(LoggingTime)).contents,
    DataTypeId.BTLE_ADDRESS: lambda p: cast(p.contents.value, POINTER(BtleAddress)).contents,
    DataTypeId.BOSCH_ANY_MOTION: lambda p: cast(p.contents.value, POINTER(BoschAnyMotion)).contents,
    DataTypeId.CALIBRATION_STATE: lambda p: cast(p.contents.value, POINTER(CalibrationState)).contents,
    DataTypeId.BOSCH_TAP: lambda p: cast(p.contents.value, POINTER(BoschTap)).contents
}

def main():
    address = "C4:A3:A4:75:A2:86"
    device = MetaWear(address)
    device.connect()

    pattern = LedPattern(repeat_count=Const.LED_REPEAT_INDEFINITELY)
    libmetawear.mbl_mw_led_load_preset_pattern(byref(pattern), LedPreset.SOLID)
    libmetawear.mbl_mw_led_write_pattern(device.board, byref(pattern), LedColor.GREEN)
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
        
        # Start time to track data
        start_time = time.time()
        
        print("Tracking data has begun")

        time.sleep(15.0)

        print("Tracking data has stopped")
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
        #print("%s    : %s -> %s" % ("Accelrometer", self.device.address, parse_value(data)))
        self. x_acc.append(parse_value(data).x)
        self.y_acc.append(parse_value(data).y)
        self.z_acc.append(parse_value(data).z)
        self.time_acc.append(time.time()-self.start_time)

    def data_handler_gyr(self, ctx, data):
        #print("%s    : %s -> %s" % ("Gyroscope", self.device.address, parse_value(data)))
        self. x_gyr.append(parse_value(data).x)
        self.y_gyr.append(parse_value(data).y)
        self.z_gyr.append(parse_value(data).z)
        self.time_gyr.append(time.time()-self.start_time)

def parse_value(pointer, **kwargs):
        """
        Helper function to extract the value from a Data object.  If you are storing the values to be used at a later time, 
        call copy.deepcopy preserve the value.  You do not need to do this if the underlying type is a native type or a byte array
        @params:
            pointer     - Required  : Pointer to a Data object
            n_elem      - Optional  : Nummber of elements in the value array if the type_id attribute is DataTypeId.DATA_ARRAY
        """
        if (pointer.contents.type_id in _value_parsers):
            return _value_parsers[pointer.contents.type_id](pointer)
        elif (pointer.contents.type_id == DataTypeId.SENSOR_ORIENTATION):
            return _value_parsers[DataTypeId.INT32](pointer)
        elif (pointer.contents.type_id == DataTypeId.BYTE_ARRAY):
            array_ptr= cast(pointer.contents.value, POINTER(c_ubyte * pointer.contents.length))
            return [array_ptr.contents[i] for i in range(0, pointer.contents.length)]
        elif (pointer.contents.type_id == DataTypeId.DATA_ARRAY):
            if 'n_elem' in kwargs:
                values = cast(pointer.contents.value, POINTER(POINTER(Data) * kwargs['n_elem']))
                return [parse_value(values.contents[i]) for i in range(0, kwargs['n_elem'])]
            else:
                raise RuntimeError("Missing optional parameter 'n_elem' for parsing DataTypeId.DATA_ARRAY value")
        else:
            raise RuntimeError('Unrecognized data type id: ' + str(pointer.contents.type_id))


        

if __name__ == "__main__":
    main()
