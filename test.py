from machine import I2C, Pin
from modules.vl53l0x import VL53L0X

i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=100000)
tof = VL53L0X(i2c)
print("Init OK")

while True:
    print(tof.read_mm(), "mm")