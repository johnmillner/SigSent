from smbus2 import SMBusWrapper
import time
import sys
import rospy
import roslib
from sigsent.msg import GPSList, Battery, Drive

roslib.load_manifest('sigsent')

class FuelGauge:
    def __init__(self):
        self.max_cap = 10000.0
        self.min_cap = 500.0

        # I2C address
        self.address = 0b1100100
        
        # Register map for device
        self.status_reg = 0x00
        self.control_reg = 0x01
        self.accum_charge_reg = 0x02
        self.voltage_reg = 0x08
        self.current_reg = 0x0e
        self.temperature_reg = 0x14

        # Threshold registers
        self.voltage_th_hi_reg = 0x0a
        self.voltage_th_lo_reg = 0x0c
        self.current_th_hi_reg = 0x10
        self.current_th_lo_reg = 0x12
        self.temperature_th_hi_reg = 0x16
        self.temperature_th_lo_reg = 0x17

        # Threshold values
        # TODO(rwales): These need setting
        self.voltage_th_hi = 16.8
        self.voltage_th_lo = 12.8
        self.current_th_hi = 50
        self.current_th_lo = 0
        self.temperature_th_hi = 100
        self.temperature_th_lo = 60

        self.control = 0b11000010

        # Set control bits and thresholds
        with SMBusWrapper(1) as bus:
            bus.write_i2c_block_data(self.address, self.control_reg, self.control)
            bus.write_i2c_block_data(self.address, self.voltage_th_hi, self.)

        self.battery_pub = rospy.Publisher('battery', Battery, queue_size=10)
        self.motor_pub = rospy.Publisher('drive', Drive, queue_size=10)

        # Read fuel gauge at 10Hz
        self.read_freq = rospy.Rate(10)
        self.update()

    def update(self):
        try:
            while True:
                self.read_gauge()
                self.read_freq.sleep()
        except rospy.ROSInterruptException:
            pass

    def read_gauge(self):
        try:
            with SMBusWrapper(1) as bus:
                voltage_readings = bus.read_i2c_block_data(self.address, self.voltage_reg, 2)
                accum_charge_readings = bus.read_i2c_block_data(self.address, self.accum_charge_reg, 2)
                current_readings = bus.read_i2c_block_data(self.address, self.current_reg, 2)
                temperature_readings = bus.read_i2c_block_data(self.address, self.temperature_reg, 2)
        except e:
            print('Error: {}'.format(e))
            sys.exit()
        
        voltage = int.from_bytes(voltage_readings, byteorder='big')
        accum_charge = int.from_bytes(accum_charge_readings, byteorder='big')
        current = int.from_bytes(current_readings, byteorder='big')
        temperature = int.from_bytes(temperature_readings, byteorder='big')

        battery = Battery()
        battery.voltage = voltage
        battery.charge = accum_charge
        battery.current = current
        battery.temperature = temperature
        battery.percent_full = accum_charge/self.max_cap

        # Turn off the motors if current too high
        if current > self.current_th_hi:
            d_msg = Drive()
            d_msg.direction.data = 0
            d_msg.speed.data = 0
            self.motor_pub.publish(d_msg)

        self.battery_pub.publish(battery)

if __name__ == '__main__':
    try:
        fg = FuelGauge()