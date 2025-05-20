import struct
import time

from machine import I2C, Pin
from MS5837 import MS5837

i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=100000)
# i2c = I2C(0, freq=399361, scl=5, sda=4, timeout=50000)
# print(i2c.scan())

sensor = MS5837(i2c)
print(sensor.init())
sensor.setModel(sensor.MS5837_30BA)
sensor.setFluidDensity(997)

fmt = "<id"
sequence = 0

while True:
    # 改行コードはCR+LFでPCから送信する
    # 115200bps
    """
    key = input()
    if key == "d":
        sensor.read()
        #print(key)
        print(sensor.depth())
        led.value(stat)
        #time.sleep(1)
        stat = 1 - stat
    """
    sequence += 1
    sensor.read()

    # to byte after to hex, and send
    s = struct.pack(fmt, sequence, sensor.depth())
    print("#" + s.hex() + "%")

    time.sleep(0.01)
    """
    print(f"depth:{sensor.depth()}")
    print(f"pressure:{sensor.pressure()}")
    print(f"temperature:{sensor.temperature()}")
    print(f"altitude:{sensor.altitude()}")
    """
