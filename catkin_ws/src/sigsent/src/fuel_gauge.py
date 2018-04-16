#!/usr/bin/python
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

        # Constants defined by the Linduino code
        self.LTC2943_CHARGE_lsb = 0.34 * (10**-3)
        self.resistor = 0.0003
        self.LTC2943_FULLSCALE_VOLTAGE = 23.6
        self.LTC2943_FULLSCALE_TEMPERATURE = 510
        self.LTC2943_FULLSCALE_CURRENT = 60 * 10**-3
        
        # Set control bits and thresholds
        with SMBusWrapper(1) as bus:
            bus.write_byte_data(self.address, self.control_reg, self.control)
            #bus.write_i2c_block_data(self.address, self.voltage_th_hi, self.voltage_th_hi_reg)

        self.battery_pub = rospy.Publisher('battery', Battery, queue_size=10)
        self.motor_pub = rospy.Publisher('drive', Drive, queue_size=10)

        # Read fuel gauge at 10Hz
        self.read_freq = rospy.Rate(2)
        self.update()

    def update(self):
        try:
            while not rospy.is_shutdown():
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

        voltage = self.code_to_voltage(voltage)
        accum_charge = self.code_to_mAh(accum_charge)
        current = self.code_to_current(current)
        temperature = self.code_to_celcius(temperature)

        battery = Battery()
        battery.voltage = voltage
        battery.charge = accum_charge
        battery.current = current
        battery.temperature = temperature
        battery.percent_full = min(100, accum_charge/self.max_cap)

        # Turn off the motors if current too high
        if current > self.current_th_hi:
            d_msg = Drive()
            d_msg.direction.data = 0
            d_msg.speed.data = 0
            self.motor_pub.publish(d_msg)

        self.battery_pub.publish(battery)

    # The conversions defined below come from the provided Linduino code by Linear
    def code_to_mAh(self, data):
        return 1000 * (data * self.LTC2943_CHARGE_lsb * 16 *50E-3)/(self.resistor * 4096)
    
    def code_to_current(self, data):
        return ((data - 32767) / (32767)) * ((self.LTC2943_FULLSCALE_CURRENT) / self.resistor)
    
    def code_to_celcius(self, data):
        return data * ((self.LTC2943_FULLSCALE_TEMPERATURE) / 65535) - 273.15

    def code_to_voltage(self, data):
        return (data / (65535)) * self.LTC2943_FULLSCALE_VOLTAGE

if __name__ == '__main__':
    try:
        fg = FuelGauge()
    except:
        pass        