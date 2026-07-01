# vl53l0x_reference.py
#
# The MIT License (MIT)
# Copyright (c) 2017 Tony DiCola for Adafruit Industries
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
# Adapted from kapetan/MicroPython_VL53L0X (fork of Adafruit CircuitPython
# VL53L0X), itself derived from the Pololu VL53L0X Arduino library and ST's
# official VL53L0X API. Used here as an independent cross-check against a
# hand-rolled driver, to isolate whether a persistent EIO during init is a
# driver bug or a wiring/hardware issue.
#
# Usage:
#   from machine import I2C, Pin
#   from vl53l0x_reference import VL53L0X
#   i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=100000)
#   tof = VL53L0X(i2c, io_timeout_ms=1000)
#   print(tof.range)

import time

_SYSRANGE_START = 0x00
_SYSTEM_SEQUENCE_CONFIG = 0x01
_SYSTEM_INTERMEASUREMENT_PERIOD = 0x04
_SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A
_GPIO_HV_MUX_ACTIVE_HIGH = 0x84
_SYSTEM_INTERRUPT_CLEAR = 0x0B
_RESULT_INTERRUPT_STATUS = 0x13
_RESULT_RANGE_STATUS = 0x14
_I2C_SLAVE_DEVICE_ADDRESS = 0x8A
_MSRC_CONFIG_CONTROL = 0x60
_PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50
_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51
_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44
_FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70
_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71
_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0
_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E
_DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F
_GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6
_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89
_MSRC_CONFIG_TIMEOUT_MACROP = 0x46
_OSC_CALIBRATE_VAL = 0xF8
_VCSEL_PERIOD_PRE_RANGE = 0
_VCSEL_PERIOD_FINAL_RANGE = 1


def _decode_timeout(val):
    return float(val & 0xFF) * (2 ** ((val & 0xFF00) >> 8)) + 1


def _encode_timeout(timeout_mclks):
    timeout_mclks = int(timeout_mclks) & 0xFFFF
    ls_byte = 0
    ms_byte = 0
    if timeout_mclks > 0:
        ls_byte = timeout_mclks - 1
        while ls_byte > 255:
            ls_byte >>= 1
            ms_byte += 1
        return ((ms_byte << 8) | (ls_byte & 0xFF)) & 0xFFFF
    return 0


def _timeout_mclks_to_us(timeout_period_mclks, vcsel_period_pclks):
    macro_period_ns = ((2304 * vcsel_period_pclks * 1655) + 500) // 1000
    return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns // 2)) // 1000


def _timeout_us_to_mclks(timeout_period_us, vcsel_period_pclks):
    macro_period_ns = ((2304 * vcsel_period_pclks * 1655) + 500) // 1000
    return ((timeout_period_us * 1000) + (macro_period_ns // 2)) // macro_period_ns


class VL53L0X:
    _BUF1 = bytearray(1)
    _BUF2 = bytearray(2)

    def __init__(self, i2c, address=0x29, io_timeout_ms=500):
        self.i2c = i2c
        self.address = address
        self.io_timeout_ms = io_timeout_ms
        self._continuous_mode = False

        if (self._r8(0xC0) != 0xEE or self._r8(0xC1) != 0xAA or self._r8(0xC2) != 0x10):
            raise RuntimeError("VL53L0X ID registers mismatch - check wiring")

        for reg, val in ((0x88, 0x00), (0x80, 0x01), (0xFF, 0x01), (0x00, 0x00)):
            self._w8(reg, val)
        self._stop_variable = self._r8(0x91)
        for reg, val in ((0x00, 0x01), (0xFF, 0x00), (0x80, 0x00)):
            self._w8(reg, val)

        self._w8(_MSRC_CONFIG_CONTROL, self._r8(_MSRC_CONFIG_CONTROL) | 0x12)
        self._w16(_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, int(0.25 * (1 << 7)))
        self._w8(_SYSTEM_SEQUENCE_CONFIG, 0xFF)

        spad_count, spad_is_aperture = self._get_spad_info()

        ref_spad_map = bytearray(self.i2c.readfrom_mem(
            self.address, _GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6))

        for reg, val in ((0xFF, 0x01), (_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00),
                          (_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C),
                          (0xFF, 0x00), (_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4)):
            self._w8(reg, val)

        first_spad_to_enable = 12 if spad_is_aperture else 0
        spads_enabled = 0
        for i in range(48):
            byte_i, bit_i = i // 8, i % 8
            if i < first_spad_to_enable or spads_enabled == spad_count:
                ref_spad_map[byte_i] &= ~(1 << bit_i) & 0xFF
            elif (ref_spad_map[byte_i] >> bit_i) & 0x1:
                spads_enabled += 1
        self.i2c.writeto_mem(self.address, _GLOBAL_CONFIG_SPAD_ENABLES_REF_0, bytes(ref_spad_map))

        tuning = [
            (0xFF, 0x01), (0x00, 0x00), (0xFF, 0x00), (0x09, 0x00), (0x10, 0x00),
            (0x11, 0x00), (0x24, 0x01), (0x25, 0xFF), (0x75, 0x00), (0xFF, 0x01),
            (0x4E, 0x2C), (0x48, 0x00), (0x30, 0x20), (0xFF, 0x00), (0x30, 0x09),
            (0x54, 0x00), (0x31, 0x04), (0x32, 0x03), (0x40, 0x83), (0x46, 0x25),
            (0x60, 0x00), (0x27, 0x00), (0x50, 0x06), (0x51, 0x00), (0x52, 0x96),
            (0x56, 0x08), (0x57, 0x30), (0x61, 0x00), (0x62, 0x00), (0x64, 0x00),
            (0x65, 0x00), (0x66, 0xA0), (0xFF, 0x01), (0x22, 0x32), (0x47, 0x14),
            (0x49, 0xFF), (0x4A, 0x00), (0xFF, 0x00), (0x7A, 0x0A), (0x7B, 0x00),
            (0x78, 0x21), (0xFF, 0x01), (0x23, 0x34), (0x42, 0x00), (0x44, 0xFF),
            (0x45, 0x26), (0x46, 0x05), (0x40, 0x40), (0x0E, 0x06), (0x20, 0x1A),
            (0x43, 0x40), (0xFF, 0x00), (0x34, 0x03), (0x35, 0x44), (0xFF, 0x01),
            (0x31, 0x04), (0x4B, 0x09), (0x4C, 0x05), (0x4D, 0x04), (0xFF, 0x00),
            (0x44, 0x00), (0x45, 0x20), (0x47, 0x08), (0x48, 0x28), (0x67, 0x00),
            (0x70, 0x04), (0x71, 0x01), (0x72, 0xFE), (0x76, 0x00), (0x77, 0x00),
            (0xFF, 0x01), (0x0D, 0x01), (0xFF, 0x00), (0x80, 0x01), (0x01, 0xF8),
            (0xFF, 0x01), (0x8E, 0x01), (0x00, 0x01), (0xFF, 0x00), (0x80, 0x00),
        ]
        for reg, val in tuning:
            self._w8(reg, val)

        self._w8(_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)
        self._w8(_GPIO_HV_MUX_ACTIVE_HIGH, self._r8(_GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10 & 0xFF)
        self._w8(_SYSTEM_INTERRUPT_CLEAR, 0x01)

        budget_us = self.get_measurement_timing_budget()
        self._w8(_SYSTEM_SEQUENCE_CONFIG, 0xE8)
        self.set_measurement_timing_budget(budget_us)

        self._w8(_SYSTEM_SEQUENCE_CONFIG, 0x01)
        self._perform_single_ref_calibration(0x40)
        self._w8(_SYSTEM_SEQUENCE_CONFIG, 0x02)
        self._perform_single_ref_calibration(0x00)
        self._w8(_SYSTEM_SEQUENCE_CONFIG, 0xE8)

    # ---- low level ----
    def _r8(self, reg):
        return self.i2c.readfrom_mem(self.address, reg, 1)[0]

    def _r16(self, reg):
        d = self.i2c.readfrom_mem(self.address, reg, 2)
        return (d[0] << 8) | d[1]

    def _w8(self, reg, val):
        self.i2c.writeto_mem(self.address, reg, bytes([val & 0xFF]))

    def _w16(self, reg, val):
        self.i2c.writeto_mem(self.address, reg, bytes([(val >> 8) & 0xFF, val & 0xFF]))

    def _w32(self, reg, val):
        self.i2c.writeto_mem(self.address, reg, bytes([
            (val >> 24) & 0xFF, (val >> 16) & 0xFF, (val >> 8) & 0xFF, val & 0xFF]))

    def _wait(self, check_fn, msg):
        start = time.ticks_ms()
        while not check_fn():
            if self.io_timeout_ms > 0 and time.ticks_diff(time.ticks_ms(), start) >= self.io_timeout_ms:
                raise RuntimeError(msg)

    # ---- init helpers ----
    def _get_spad_info(self):
        for reg, val in ((0x80, 0x01), (0xFF, 0x01), (0x00, 0x00), (0xFF, 0x06)):
            self._w8(reg, val)
        self._w8(0x83, self._r8(0x83) | 0x04)
        for reg, val in ((0xFF, 0x07), (0x81, 0x01), (0x80, 0x01), (0x94, 0x6B), (0x83, 0x00)):
            self._w8(reg, val)

        self._wait(lambda: self._r8(0x83) != 0x00, "Timeout waiting for VL53L0X (SPAD info)")

        self._w8(0x83, 0x01)
        tmp = self._r8(0x92)
        count = tmp & 0x7F
        is_aperture = ((tmp >> 7) & 0x01) == 1
        for reg, val in ((0x81, 0x00), (0xFF, 0x06)):
            self._w8(reg, val)
        self._w8(0x83, self._r8(0x83) & ~0x04 & 0xFF)
        for reg, val in ((0xFF, 0x01), (0x00, 0x01), (0xFF, 0x00), (0x80, 0x00)):
            self._w8(reg, val)
        return count, is_aperture

    def _perform_single_ref_calibration(self, vhv_init_byte):
        self._w8(_SYSRANGE_START, 0x01 | (vhv_init_byte & 0xFF))
        self._wait(lambda: (self._r8(_RESULT_INTERRUPT_STATUS) & 0x07) != 0,
                   "Timeout waiting for VL53L0X (ref calibration)")
        self._w8(_SYSTEM_INTERRUPT_CLEAR, 0x01)
        self._w8(_SYSRANGE_START, 0x00)

    def _get_vcsel_pulse_period(self, vcsel_period_type):
        if vcsel_period_type == _VCSEL_PERIOD_PRE_RANGE:
            val = self._r8(_PRE_RANGE_CONFIG_VCSEL_PERIOD)
        else:
            val = self._r8(_FINAL_RANGE_CONFIG_VCSEL_PERIOD)
        return ((val + 1) & 0xFF) << 1

    def _get_sequence_step_enables(self):
        cfg = self._r8(_SYSTEM_SEQUENCE_CONFIG)
        tcc = (cfg >> 4) & 0x1 > 0
        dss = (cfg >> 3) & 0x1 > 0
        msrc = (cfg >> 2) & 0x1 > 0
        pre_range = (cfg >> 6) & 0x1 > 0
        final_range = (cfg >> 7) & 0x1 > 0
        return tcc, dss, msrc, pre_range, final_range

    def _get_sequence_step_timeouts(self, pre_range):
        pre_vcsel = self._get_vcsel_pulse_period(_VCSEL_PERIOD_PRE_RANGE)
        msrc_dss_tcc_mclks = (self._r8(_MSRC_CONFIG_TIMEOUT_MACROP) + 1) & 0xFF
        msrc_dss_tcc_us = _timeout_mclks_to_us(msrc_dss_tcc_mclks, pre_vcsel)

        pre_range_mclks = _decode_timeout(self._r16(_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI))
        pre_range_us = _timeout_mclks_to_us(pre_range_mclks, pre_vcsel)

        final_vcsel = self._get_vcsel_pulse_period(_VCSEL_PERIOD_FINAL_RANGE)
        final_range_mclks = _decode_timeout(self._r16(_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI))
        if pre_range:
            final_range_mclks -= pre_range_mclks
        final_range_us = _timeout_mclks_to_us(final_range_mclks, final_vcsel)

        return msrc_dss_tcc_us, pre_range_us, final_range_us, final_vcsel, pre_range_mclks

    def get_measurement_timing_budget(self):
        budget_us = 1910 + 960
        tcc, dss, msrc, pre_range, final_range = self._get_sequence_step_enables()
        msrc_dss_tcc_us, pre_range_us, final_range_us, _, _ = self._get_sequence_step_timeouts(pre_range)
        if tcc:
            budget_us += msrc_dss_tcc_us + 590
        if dss:
            budget_us += 2 * (msrc_dss_tcc_us + 690)
        elif msrc:
            budget_us += msrc_dss_tcc_us + 660
        if pre_range:
            budget_us += pre_range_us + 660
        if final_range:
            budget_us += final_range_us + 550
        return budget_us

    def set_measurement_timing_budget(self, budget_us):
        used_budget_us = 1320 + 960
        tcc, dss, msrc, pre_range, final_range = self._get_sequence_step_enables()
        msrc_dss_tcc_us, pre_range_us, _, final_vcsel, pre_range_mclks = \
            self._get_sequence_step_timeouts(pre_range)
        if tcc:
            used_budget_us += msrc_dss_tcc_us + 590
        if dss:
            used_budget_us += 2 * (msrc_dss_tcc_us + 690)
        elif msrc:
            used_budget_us += msrc_dss_tcc_us + 660
        if pre_range:
            used_budget_us += pre_range_us + 660
        if final_range:
            used_budget_us += 550
            if used_budget_us > budget_us:
                raise ValueError("Requested timeout too big")
            final_range_timeout_us = budget_us - used_budget_us
            final_range_timeout_mclks = _timeout_us_to_mclks(final_range_timeout_us, final_vcsel)
            if pre_range:
                final_range_timeout_mclks += pre_range_mclks
            self._w16(_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, _encode_timeout(final_range_timeout_mclks))

    # ---- ranging ----
    @property
    def range(self):
        return self.read_range_single_millimeters()

    def read_range_single_millimeters(self):
        for reg, val in ((0x80, 0x01), (0xFF, 0x01), (0x00, 0x00), (0x91, self._stop_variable),
                          (0x00, 0x01), (0xFF, 0x00), (0x80, 0x00), (_SYSRANGE_START, 0x01)):
            self._w8(reg, val)
        self._wait(lambda: (self._r8(_SYSRANGE_START) & 0x01) == 0,
                   "Timeout waiting for VL53L0X (range start)")
        return self._read_range_continuous()

    def _read_range_continuous(self):
        self._wait(lambda: (self._r8(_RESULT_INTERRUPT_STATUS) & 0x07) != 0,
                   "Timeout waiting for VL53L0X (range result)")
        range_mm = self._r16(_RESULT_RANGE_STATUS + 10)
        self._w8(_SYSTEM_INTERRUPT_CLEAR, 0x01)
        return range_mm