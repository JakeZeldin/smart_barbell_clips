from mbientlab.metawear import MetaWear, libmetawear
from mbientlab.metawear.cbindings import *
import time

def main():
    address = "C7:EA:21:57:F5:E2"
    device = MetaWear(address)
    device.connect()

    pattern = LedPattern(repeat_count=Const.LED_REPEAT_INDEFINITELY)
    libmetawear.mbl_mw_led_load_preset_pattern(byref(pattern), LedPreset.SOLID)
    libmetawear.mbl_mw_led_write_pattern(device.board, byref(pattern), LedColor.GREEN)
    libmetawear.mbl_mw_led_play(device.board)

    time.sleep(5.0)
    libmetawear.mbl_mw_led_stop_and_clear(device.board)
    time.sleep(1.0)

    device.disconnect()

if __name__ == "__main__":
    main()
