import _thread
import time
from machine import Pin, I2C, PWM
from MS5837 import MS5837

i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=50000)
#i2c = I2C(0, freq=399361, scl=5, sda=4, timeout=50000)
#print(i2c.scan())
#i2c = I2C(0, freq=100000)
sensor = MS5837(i2c)
print(sensor.init())
sensor.setModel(sensor.MS5837_30BA)
#sensor.setModel(sensor.MS5837_30BA)
sensor.setFluidDensity(997)
led = Pin(25, Pin.OUT)
stat = 0
while True:
    # 改行コードはCR+LFでPCから送信する
    #115200bps
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
    sensor.read()
    #print(key)
    print(sensor.depth())
    """
    print(f"depth:{sensor.depth()}")
    print(f"pressure:{sensor.pressure()}")
    print(f"temperature:{sensor.temperature()}")
    print(f"altitude:{sensor.altitude()}")
    """
    #led.value(stat)
    time.sleep(0.05) 

