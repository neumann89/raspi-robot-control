#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ---------------------------------------------
# Project:  AMSpi class
# Author:   Jan Lipovský, 2016
#           Daniel Neumann, 2017
# E-mail:   janlipovsky@gmail.com
# Licence:  MIT
# Description: Python class for controlling
# Arduino Motor Shield L293D from Raspberry Pi
# ---------------------------------------------
try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error importing RPi.GPIO! This is probably because you need superuser privileges. "
          "You can achieve this by using 'sudo' to run your script")


class AMSpi:
    """
    Main class for controlling Arduino Motor Shield L293D
    via Raspberry Pi GPIO
    """
    # Motor numbering
    DC_Motor_1 = 1
    DC_Motor_2 = 2
    DC_Motor_3 = 3
    DC_Motor_4 = 4

    # Shift register
    _DIR_LATCH = None
    _DIR_CLK = None
    _DIR_SER = None

    # DC Motors states and settings
    # pin               - PIN on which is DC Motor connected
    # direction         - List of numbers that set direction of motor (clockwise, counterclockwise, no spin)
    # is_running        - True if motor is running
    # running_direction - Direction of the motor
    # pwm_frequency     - Frequency of pulse-width modulation (pwm)
    # pwm               - Un-/set pwm object
    _MOTORS = {
        DC_Motor_1: {"pin": None, "direction": [4, 8, 4 | 8], "is_running": False,
                        "running_direction": None, "pwm_frequency": 50, "pwm": None},
        DC_Motor_2: {"pin": None, "direction": [2, 16, 2 | 16], "is_running": False,
                        "running_direction": None, "pwm_frequency": 50, "pwm": None},
        DC_Motor_3: {"pin": None, "direction": [32, 128, 32 | 128], "is_running": False,
                        "running_direction": None, "pwm_frequency": 50, "pwm": None},
        DC_Motor_4: {"pin": None, "direction": [1, 64, 1 | 64], "is_running": False,
                        "running_direction": None, "pwm_frequency": 50, "pwm": None}
    }

    # indexes of motor direction list
    _clockwise = 0
    _counterclockwise = 1
    _stop = 2

    def __init__(self, use_board=False):
        """
        Initialize function for AMSpi class

        :param bool use_board: True if GPIO.BOARD numbering will be used
        :return:
        """
        if use_board:
            GPIO.setmode(GPIO.BOARD)
            print("PIN numbering: BOARD")
        else:
            GPIO.setmode(GPIO.BCM)
            print("PIN numbering: BCM")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        try:
            self._shift_write(0)
            self.stop_dc_motors([self.DC_Motor_1, self.DC_Motor_2, self.DC_Motor_3, self.DC_Motor_4])
            GPIO.cleanup()
        except RuntimeWarning:
            return True

    def _test_shift_pins(self):
        """
        Test if PINs of shift register were set

        :return: bool True if test passed
        :rtype bool
        """
        if self._DIR_LATCH is None:
            return False
        if self._DIR_CLK is None:
            return False
        if self._DIR_SER is None:
            return False

        return True

    def _shift_write(self, value):
        """
        Write given value to the shift register
        :param int value: value which you want to write to shift register
        :return:
        """
        if self._test_shift_pins() is False:
            print("ERROR: PINs for shift register were not set properly.")
            self.__exit__(None, None, None)

        GPIO.output(self._DIR_LATCH, GPIO.LOW)
        for x in range(0, 8):
            temp = value & 0x80
            GPIO.output(self._DIR_CLK, GPIO.LOW)
            if temp == 0x80:
                # data bit HIGH
                GPIO.output(self._DIR_SER, GPIO.HIGH)
            else:
                # data bit LOW
                GPIO.output(self._DIR_SER, GPIO.LOW)
            GPIO.output(self._DIR_CLK, GPIO.HIGH)
            value <<= 0x01  # shift left

        GPIO.output(self._DIR_LATCH, GPIO.HIGH)

    def set_74HC595_pins(self, DIR_LATCH, DIR_CLK, DIR_SER):
        """
        Set PINs used on Raspberry Pi to connect with 74HC595 module on
        Arduino Motor Shield
        :param int DIR_LATCH: LATCH PIN number
        :param int DIR_CLK: CLK PIN number
        :param int DIR_SER: SER  PIN number
        :return:
        """
        self._DIR_LATCH = DIR_LATCH
        self._DIR_CLK = DIR_CLK
        self._DIR_SER = DIR_SER

        GPIO.setup(self._DIR_LATCH, GPIO.OUT)
        GPIO.setup(self._DIR_CLK, GPIO.OUT)
        GPIO.setup(self._DIR_SER, GPIO.OUT)

    def set_L293D_pins(self, PWM0A=None, PWM0B=None, PWM2A=None, PWM2B=None):
        """
        Set PINs used on Raspberry Pi to connect with 74HC595 module on
        Arduino Motor Shield

        :param int PWM0A: PWM0A PIN number
        :param int PWM0B: PWM0B PIN number
        :param int PWM2A: PWM2A PIN number
        :param int PWM2B: PWM2B PIN number
        """
        # self.PWM0A = PWM0A
        self._MOTORS[self.DC_Motor_4]["pin"] = PWM0B
        # self.PWM0B = PWM0B
        self._MOTORS[self.DC_Motor_3]["pin"] = PWM0A
        # self.PWM2A = PWM2A
        self._MOTORS[self.DC_Motor_1]["pin"] = PWM2A
        # self.PWM2B = PWM2B
        self._MOTORS[self.DC_Motor_2]["pin"] = PWM2B

        if PWM0A is not None:
            GPIO.setup(PWM0A, GPIO.OUT)
        if PWM0B is not None:
            GPIO.setup(PWM0B, GPIO.OUT)
        if PWM2A is not None:
            GPIO.setup(PWM2A, GPIO.OUT)
        if PWM2B is not None:
            GPIO.setup(PWM2B, GPIO.OUT)

    def _get_motors_direction(self, dc_motor, directions_index):
        """
        Compute number that should be writen to shift register to run/stop motor

        :param int dc_motor: number of dc motor
        :param int directions_index: index to motor direction list
        :return: number for shift register, motors direction value
        :rtype tuple
        """

        direction_value = self._MOTORS[dc_motor]["direction"][directions_index]
        all_motors_direction = direction_value
        for tmp_dc_motor in [self.DC_Motor_1, self.DC_Motor_2, self.DC_Motor_3, self.DC_Motor_4]:
            if tmp_dc_motor == dc_motor:
                continue
            if self._MOTORS[tmp_dc_motor]["running_direction"] is not None:
                all_motors_direction += self._MOTORS[tmp_dc_motor]["running_direction"]

        return all_motors_direction, direction_value

    def set_pwm_frequency(self, frequencies):
        """
        Sets the pulse-width modulation (pwm) frequencies for each motor.

        :param list of int: Values for pwm frequencies. Should be high enough to run smoothly, but
                            too high values can cause RPi.GPIO to crash.
        """
        if size(frequencies) is not size(self._MOTORS):
            print("ERROR: Size of frequency list must be equal to size of motors ("+size(self._MOTORS)+")!")
        else:
            for i in range(0,size(frequencies)):
                self._MOTORS[i]["pwm_frequency"] = frequencies[i]

    def get_pwm_frequency(self):
        """
        Returns the current list of pulse-width modulation (pwm) frequencies for each motor.

        :return: Current pwm frequency for each motor as list.
        :rtype list of int
        """
        frequencies = []
        for m in self._MOTORS:
            frequencies += m["pwm_frequency"]

        return frequencies

    def run_dc_motor(self, dc_motor, clockwise=True, speed=None):
        """
        Run motor with given direction

        :param int dc_motor: number of dc motor
        :param bool clockwise: True for clockwise False for counterclockwise
        :param int speed: pwm duty cycle (range 0-100)
        :return: False in case of an ERROR, True if everything is OK
        :rtype bool
        """
        if self._MOTORS[dc_motor]["pin"] is None:
            print("WARNING: Pin for DC_Motor_{} is not set. Can not run motor.".format(dc_motor))
            return False

        all_motors_direction, direction_value = self._get_motors_direction(dc_motor, int(not clockwise))

        # set motors direction
        self._shift_write(all_motors_direction)

        # turn the motor on (if speed argument is not given then full speed, otherwise set pwm according to speed)
        if speed is None:
            GPIO.output(self._MOTORS[dc_motor]["pin"], GPIO.HIGH)
        elif speed >= 0 and speed <= 100:
            self._MOTORS[dc_motor]["pwm"] = GPIO.PWM(self._MOTORS[dc_motor]["pin"],
                                                     self._MOTORS[dc_motor]["pwm_frequency"])
            self._MOTORS[dc_motor]["pwm"].start(speed)
        else:
            print("WARNING: Speed argument must be in range 0-100! " + str(speed) + " given.")
            return False

        self._MOTORS[dc_motor]["is_running"] = True
        self._MOTORS[dc_motor]["running_direction"] = direction_value

        return True

    def run_dc_motors(self, dc_motors, clockwise=True, speed=None):
        """
        Run motors with given direction

        :param list[int] dc_motors: list of dc motor numbers
        :param bool clockwise: True for clockwise, False for counterclockwise
        :param int speed: pwm duty cycle (range 0-100)
        :return: False in case of an ERROR, True if everything is OK
        :rtype bool
        """
        for dc_motor in dc_motors:
            self.run_dc_motor(dc_motor, clockwise, speed)

    def stop_dc_motor(self, dc_motor):
        """
        Stop running motor

        :param int dc_motor: number of dc motor
        :return: False in case of an ERROR, True if everything is OK
        :rtype bool
        """
        if self._MOTORS[dc_motor]["pin"] is None:
            # print("WARNING: Pin for DC_Motor_{} is not set. Stopping motor could not be done".format(dc_motor))
            return False

        all_motors_direction, direction_value = self._get_motors_direction(dc_motor, self._stop)
        # set motors direction
        self._shift_write(all_motors_direction)

        if self._MOTORS[dc_motor]["pwm"] is None:
            GPIO.output(self._MOTORS[dc_motor]["pin"], GPIO.LOW)
        else:
            self._MOTORS[dc_motor]["pwm"].stop()
            self._MOTORS[dc_motor]["pwm"] = None

        self._MOTORS[dc_motor]["is_running"] = False
        self._MOTORS[dc_motor]["running_direction"] = None

        return True

    def stop_dc_motors(self, dc_motors):
        """
        Stop motors set in list

        :param list[int] dc_motors: list of dc motor numbers
        :return: False in case of an ERROR, True if everything is OK
        :rtype bool
        """
        for dc_motor in dc_motors:
            if not self.stop_dc_motor(dc_motor):
                return False

        return True

