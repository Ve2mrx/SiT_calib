#!/bin/python3
# 2024-12-07 20:30

import datetime
# import math
import struct
# import time

# import smbus


class SiT5721:
    def __init__(self, bus, address):
        self.bus = bus
        self.address = address
        self.read_SiT_static()
        self.read_SiT_config()
        self.read_SiT_operation()
        self.read_SiT_dynamic()

        self.calc_SiT_current_compensation()
        self.calc_SiT_new_pull_value_from_target(target_pull_value=0.0000000)

    def read_SiT_static(self):
        # From SiTime datasheet for SiT5721, rev 1.0

        # 0x50, Part Number, 256 bytes, ASCII, , R, Factory Set
        # ***Note that smbus cannot read 256 bytes, so set to 32 bytes***
        self.part_num_str = ''.join([chr(item) for item in
                                     self.bus.read_i2c_block_data(self.address, 0x50, 32)])

        # 0x52, Nominal Frequency, 32 bytes, ASCII, MHz, R, Factory Set
        self.nomfreq_str = ''.join([chr(item) for item in
                                    self.bus.read_i2c_block_data(self.address, 0x52, 32)])

        # 0x56, Lot and Serial Numbers, 32 bytes, ASCII, , R, Factory Set
        self.lot_sn_str = ''.join([chr(item) for item in
                                   self.bus.read_i2c_block_data(self.address, 0x56, 32)])

        # 0x57, Fabrication Date, 32 bytes, ASCII, , R, Factory Set
        self.fab_date_str = ''.join([chr(item) for item in
                                     self.bus.read_i2c_block_data(self.address, 0x57, 32)])

    def read_SiT_config(self):
        # From SiTime datasheet for SiT5721, rev 1.0

        # 0x61, Pull Value, 4 bytes, Float, ± Fractional Offset, R/W, Default 0
        self.pull_value = float(''.join([str(item) for item in
                                         struct.unpack('f', bytes(
                                             self.bus.read_i2c_block_data(self.address, 0x61, 4)))
                                         ]))

        # 0x62, Pull Range, 4 bytes, Float, Fractional Offset, R/W, Default 10E-06
        self.pull_range = float(''.join([str(item) for item in
                                         struct.unpack('f', bytes(
                                             self.bus.read_i2c_block_data(self.address, 0x62, 4)))
                                         ]))

        # 0x63, Aging Compensation, 4 bytes, Float, ± Fractional Offset, R/W, Default 0
        self.aging_compensation = float(''.join([str(item) for item in
                                                 struct.unpack('f', bytes(
                                                     self.bus.read_i2c_block_data(self.address, 0x63, 4)))
                                                 ]))

        # 0x64, Max Freq. Ramp Rate, 4 bytes, Float, Fractional Offset, R/W, Default 1.00E-05
        self.max_freq_ramp_rate = float(''.join([str(item) for item in
                                                 struct.unpack('f', bytes(
                                                     self.bus.read_i2c_block_data(self.address, 0x64, 4)))
                                                 ]))

    def read_SiT_operation(self):
        # From SiTime datasheet for SiT5721, rev 1.0
        # Read less critical values
        # -- 0xAB, Resonator Temp., 4 bytes, Float, Degrees C, R
        self.temperature_float = float(''.join([str(item) for item in
                                                struct.unpack('f', bytes(
                                                    self.bus.read_i2c_block_data(self.address, 0xA1, 4)))
                                                ]))

        # -- 0xA3, Micro. Supply Voltage, 4 bytes, Float, Volts, R
        self.supply_voltage_float = float(''.join([str(item) for item in
                                                   struct.unpack('f', bytes(
                                                       self.bus.read_i2c_block_data(self.address, 0xA3, 4)))
                                                   ]))

        # -- 0xA7, Heater Power, 4 bytes, Float, Watts, R
        self.heater_power_float = float(''.join([str(item) for item in
                                                 struct.unpack('f', bytes(
                                                     self.bus.read_i2c_block_data(self.address, 0xA7, 4)))
                                                 ]))

        # -- 0xB0, Temp. Error, 4 bytes, Float, Degrees C, R
        self.temperature_err_float = float(''.join([str(item) for item in
                                                    struct.unpack('f', bytes(
                                                        self.bus.read_i2c_block_data(self.address, 0xB0, 4)))
                                                    ]))

        # -- 0xB1, Power Target, 4 bytes, Float, Watts, R
        self.heater_power_target_float = float(''.join([str(item) for item in
                                                        struct.unpack('f', bytes(
                                                            self.bus.read_i2c_block_data(self.address, 0xB1, 4)))
                                                        ]))

    def read_SiT_dynamic(self):
        # From SiTime datasheet for SiT5721, rev 1.0
        # Read critical values
        # -- 0xA0, Time Since Power Up, 4 bytes, Unsigned Int, Seconds, R
        self.uptime_uint = int(''.join([str(item) for item in
                                        struct.unpack('I', bytes(
                                            self.bus.read_i2c_block_data(self.address, 0xA0, 4)))
                                        ]))

        # -- 0xAB, Total Offset Written, 4 bytes, Float, ± Fractional Offset, R
        self.total_offset_written = float(''.join([str(item) for item in
                                                   struct.unpack('f', bytes(
                                                       self.bus.read_i2c_block_data(self.address, 0xAB, 4)))
                                                   ]))

        # -- 0xAE, Error Status Flag, 4 bytes, Unsigned Int, Bit Field, R
        self.error_status_flag_uint = int(''.join([str(item) for item in
                                                   struct.unpack('I', bytes(
                                                       self.bus.read_i2c_block_data(self.address, 0xAE, 4)))
                                                   ]))

        # -- 0xAF, Stability Flag, 4 bytes, Unsigned Int, , R
        self.stability_flag_uint = int(''.join([str(item) for item in
                                                struct.unpack('I', bytes(
                                                    self.bus.read_i2c_block_data(self.address, 0xAF, 4)))
                                                ]))

        # Create status strings from flags
        self.error_status_str = "undefined"
        self.stability_status_str = "undefined"

        def error_status(flag):
            # print(flag)
            if (flag == 7):
                error_status = "good"
            else:
                error_status = "ERROR"
            return error_status

        def stability_status(flag):
            # print(flag)
            if (flag == 1):
                error_status = "stabilized"
            else:
                error_status = "unstabilized"
            return error_status

        self.error_status_str = error_status(self.error_status_flag_uint)
        self.stability_status_str = stability_status(self.stability_flag_uint)

    def calc_SiT_current_compensation(self):
        # self.read_SiT_config()
        # self.read_SiT_dynamic()

        self.current_compensation = self.total_offset_written - self.pull_value
        # print("Current Compensation   {:=+.8g} part/s".format(self.current_compensation), end='\n')
        return self.current_compensation

    def calc_SiT_new_pull_value_from_target(self, target_pull_value):

        self.new_pull_value = target_pull_value - self.calc_SiT_current_compensation()
        # print("Target Pull Value      {:=+.8g} ppm".format(target_pull_value / pow(10, -6)), end='\n')
        # print("New Pull Value         {:=+.8g} ppm".format(self.new_pull_value / pow(10, -6)), end='\n')

        return self.new_pull_value

    def set_pull_value_from_target(self, target_pull_value):
        self.set_pull_value(
            self.calc_SiT_new_pull_value_from_target(target_pull_value))

    def set_pull_value(self, new_pull_value):
        self.bus.write_i2c_block_data(self.address, 0x61, list(
            struct.pack('f', new_pull_value)))

    def set_pull_range(self, new_pull_range):
        self.bus.write_i2c_block_data(self.address, 0x62, list(
            struct.pack('f', new_pull_range)))

    def set_aging_comp(self, new_aging_compensation):
        self.bus.write_i2c_block_data(self.address, 0x63, list(
            struct.pack('f', new_aging_compensation)))

    def set_max_freq_ramp_rate(self, new_max_freq_ramp_rate):
        self.bus.write_i2c_block_data(self.address, 0x64, list(
            struct.pack('f', new_max_freq_ramp_rate)))

    def reset_error(self):
        self.bus.write_i2c_block_data(
            self.address, 0xE1, list(0x64, 0x01))  # Will it work?

    def print_SiT_static(self):
        # self.read_SiT_static()

        print("Part Number            ", self.part_num_str, end='\n')
        print("Nominal frequency      ", self.nomfreq_str, end='\n')
        print("Lot-SN                 ", self.lot_sn_str, end='\n')
        print("Fabrication            ", self.fab_date_str, end='\n')
        print()

    def print_SiT_operation(self):
        # self.read_SiT_operation()

        print("Supply voltage          {:.8g} V".format(
            self.supply_voltage_float), end='\n')
        print("Resonator temperature {:=3.8g}°C".format(
            self.temperature_float), end='\n')
        print(
            "Temperature error      {:=+3.8g}°C".format(self.temperature_err_float), end='\n')
        print("Heater power            {:.8g} W".format(
            self.heater_power_float), end='\n')
        print("Target power            {:.8g} W".format(
            self.heater_power_target_float), end='\n')
        print()

    def print_SiT_dynamic(self):
        # self.read_SiT_dynamic()
        # self.read_SiT_config()

        print("Uptime                {:8d}s, {}".format(
            self.uptime_uint,
            datetime.timedelta(seconds=self.uptime_uint)
        ), end='\n')
        print()
        print("Error status flag      ", self.error_status_str, end='\n')
        print("Stability flag         ", self.stability_status_str, end='\n')
        print()
        print(
            "Pull Value             {:=+.8g} ppm".format(self.pull_value / pow(10, -6)), end='\n')
        print("Pull Range              {:=.8g} ppm".format(
            self.pull_range / pow(10, -6)), end='\n')
        print(
            "Aging compensation     {:=+.8g} part/s".format(self.aging_compensation), end='\n')
        # print("Aging compensation     {:=+.8g} ppb/s".format(self.aging_compensation / pow(10, -9)), end='\n')
        print("Max. Freq Ramp Rate     {:=.8g} ppm".format(
            self.max_freq_ramp_rate / pow(10, -6)), end='\n')
        print()
        print("Total offset written   {:=+.8g} ppm".format(
            self.total_offset_written / pow(10, -6)), end='\n')

    def print_SiT_derived(self):
        print(
            "Target Pull Value      {:=+.8g} ppm".format(target_pull_value / pow(10, -6)), end='\n')
        print("Current Compensation   {:=+.8g} part/s".format(
            self.calc_SiT_current_compensation(), end='\n'))
        print("New Pull Value         {:=+.8g} ppm".format(
            self.calc_SiT_new_pull_value_from_target(target_pull_value) / pow(10, -6)),
            end='\n')

    def print_SiT_short(self):
        # self.read_SiT_dynamic()
        # self.read_SiT_config()

        print("Error status flag      ", self.error_status_str, end='\n')
        print(
            "Pull Value             {:=+.8g} ppm".format(self.pull_value / pow(10, -6)), end='\n')
        print("Pull Range              {:=.8g} ppm".format(
            self.pull_range / pow(10, -6)), end='\n')
        print(
            "Aging compensation     {:=+.8g} part/s".format(self.aging_compensation), end='\n')
        # print("Aging compensation     {:=+.8g} ppb/s".format(self.aging_compensation / pow(10, -9)), end='\n')
        print("Max. Freq Ramp Rate     {:=.8g} ppm".format(
            self.max_freq_ramp_rate / pow(10, -6)), end='\n')
        print()
        print("Total offset written   {:=+.8g} ppm".format(
            self.total_offset_written / pow(10, -6)), end='\n')
