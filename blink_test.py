from mbientlab.metawear import MetaWear, libmetawear
from mbientlab.metawear.cbindings import *
import time

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
        
    def __init__(self,device):
        
        self.device = device
        # Callback function pointer
        callback = FnVoid_VoidP_DataP(self.data_handler)
        
        # Setup the accelerometer sample frequency and range
        libmetawear.mbl_mw_acc_set_odr(device.board, 100.0)
        libmetawear.mbl_mw_acc_set_range(device.board, 16.0)
        libmetawear.mbl_mw_acc_write_acceleration_config(device.board)

        # Get the accelerometer data signal
        signal = libmetawear.mbl_mw_acc_get_acceleration_data_signal(device.board)
        # Subscribe to it
        libmetawear.mbl_mw_datasignal_subscribe(signal, None, callback)

        # Enable the accelerometer
        libmetawear.mbl_mw_acc_enable_acceleration_sampling(device.board)
        libmetawear.mbl_mw_acc_start(device.board)
        
        time.sleep(15.0)


        # Disable the accelerometer
        libmetawear.mbl_mw_acc_stop(device.board)
        libmetawear.mbl_mw_acc_disable_acceleration_sampling(device.board)

        # Unsubscribe to it
        signal = libmetawear.mbl_mw_acc_get_acceleration_data_signal(device.board)
        libmetawear.mbl_mw_datasignal_unsubscribe(signal)
        libmetawear.mbl_mw_debug_disconnect(device.board)



    # Callback function to process/parse the gyroscope data
    def data_handler(self, ctx, data):
        print("%s -> %s" % (self.device.address, parse_value(data)))

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
