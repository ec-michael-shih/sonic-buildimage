#!/usr/bin/env python
# -*- coding: utf-8 -*
# Copyright (c) 2019 Edgecore Networks Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may
# not use this file except in compliance with the License. You may obtain
# a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
#
# THIS CODE IS PROVIDED ON AN  *AS IS* BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
# LIMITATION ANY IMPLIED WARRANTIES OR CONDITIONS OF TITLE, FITNESS
# FOR A PARTICULAR PURPOSE, MERCHANTABLITY OR NON-INFRINGEMENT.
#
# See the Apache Version 2.0 License for specific language governing
# permissions and limitations under the License.
#
# HISTORY:
#    mm/dd/yyyy (A.D.)#
#    07/12/2022: Michael_Shih create for as9736_64d thermal plan
# ------------------------------------------------------------------

try:
    import subprocess
    import getopt
    import sys
    import logging
    import logging.config
    import logging.handlers
    import signal
    import time  # this is only being used as part of the example
    from as9736_64d.fanutil import FanUtil
    from as9736_64d.thermalutil import ThermalUtil
except ImportError as e:
    raise ImportError('%s - required module not found' % str(e))

VERSION = '1.0'
FUNCTION_NAME = '/usr/local/bin/accton_as9736_64d_monitor'

class switch(object):
    def __init__(self, value):
        self.value = value
        self.fall = False

    def __iter__(self):
        """Return the match method once, then stop"""
        yield self.match
        raise StopIteration

    def match(self, *args):
        """Indicate whether or not to enter a case suite"""
        if self.fall or not args:
            return True
        elif self.value in args: # changed for v1.5, see below
            self.fall = True
            return True
        else:
            return False


# Thermal policy(AFO-Front to back):
#
# Fan Speed Level:
#     Fan Level 1 (Fan speed:  50%, RPM +/- 10%:  6800)
#     Fan Level 2 (Fan speed:  70%, RPM +/- 10%:  9520)
#     Fan Level 3 (Fan speed: 100%, RPM +/- 10%: 13600)
#
# Using Thermal sensors as below:
#    - SMB TMP75 (0X48)
#    - UDB TMP75 (0X49)
#    - UDB TMP422(0X48)
#    - LDB TMP422(0X4d)
#    - TH4 TMP422(0X4c)
#    - CPU core_1~9
#    - MAC core_1~15
#
# Raise to Fan Level 2 from Level 1 condition:
#    - SMB TMP75 (0X48) + UDB TMP75 (0X49) + UDB TMP422(0X48) + LDB TMP422(0X4d) >= 224
#      or
#    - TH4 TMP422(0X4c) >= 92
#
# Slow down to Fan Level 1 from Level 2 condition:
#    - SMB TMP75 (0X48) + UDB TMP75 (0X49) + UDB TMP422(0X48) + LDB TMP422(0X4d) <= 211
#      and
#    - TH4 TMP422(0X4c) <= 83
#
# Raise to Fan Level 3 conditions:
#    - Fan failed
#    - CPU core temp between 80 and 99 degree C
#    - MAC temp between 85 and 100 degree C
#
# Slow down to Fan Level 1(Normal state) from Level 3 condition:
#    - CPU core temp cool down to 80 degree C
#    - MAC core temp cool down to 85 degree C
#

fan_policy_state = 0
fan_fail = 0
count_check = 0
aaa = 0

board_thermal_val_total = 0
cpu_fan_policy_state = 0

exit_by_sigterm=0

def power_off_dut():
    # Need to implement!!
    return True

# Make a class we can use to capture stdout and sterr in the log
class device_monitor(object):
    # static temp var
    init_duty_cycle = 0
    new_duty_cycle = 0
    ori_duty_cycle = 0

    def __init__(self, log_file, log_level):
        """Needs a logger and a logger level."""

        self.thermal = ThermalUtil()
        self.fan = FanUtil()

        # set up logging to file
        logging.basicConfig(
            filename=log_file,
            filemode='w',
            level=log_level,
            format= '[%(asctime)s] {%(pathname)s:%(lineno)d} %(levelname)s - %(message)s',
            datefmt='%H:%M:%S'
        )
        # set up logging to console
        if log_level == logging.DEBUG:
            console = logging.StreamHandler()
            console.setLevel(log_level)
            formatter = logging.Formatter('%(name)-12s: %(levelname)-8s %(message)s')
            console.setFormatter(formatter)
            logging.getLogger('').addHandler(console)

        sys_handler = logging.handlers.SysLogHandler(address = '/dev/log')
        sys_handler.setLevel(logging.WARNING)
        logging.getLogger('').addHandler(sys_handler)
        #logging.debug('SET. logfile:%s / loglevel:%d', log_file, log_level)

    def set_fan_duty_cycle(self, fan_level, duty_cycle_percentage):
        logging.debug("- [Fan]: fan_policy_state = %d, set new_duty_cycle = %d", fan_level, duty_cycle_percentage)
        self.fan.set_fan_duty_cycle(duty_cycle_percentage)

    def manage_fans(self):

        global fan_policy_state
        global fan_fail
        global count_check
        global board_thermal_val_total
        global board_thermal_th4_tmp422
        global cpu_fan_policy_state
        global mac_fan_policy_state

        CHECK_TIMES=3

        LEVEL_FAN_INIT=0
        FAN_LEVEL_1 = 1
        FAN_LEVEL_2 = 2
        FAN_LEVEL_3 = 3
        POLICY_NEED_SHUTDOWN  = 4

        fan_speed_policy = {
            FAN_LEVEL_1: [50],
            FAN_LEVEL_2: [70],
            FAN_LEVEL_3: [100]
        }

        thermal_spec={
            "min_to_mid_temp" : [224000, 92000],
            "mid_to_min_temp" : [211000, 83000],
            "cpu_temp"        : [ 80000, 99000],
            "mac_temp"        : [ 85000, 100000]
        }

        board_thermal_val   = [0, 0, 0, 0, 0]
        cpucore_thermal_val = [0, 0, 0, 0, 0, 0, 0, 0]
        mactemp_thermal_val = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        fan = self.fan
        thermal = self.thermal

        # Get duty_cycle at init:
        if fan_policy_state == LEVEL_FAN_INIT:
            self.init_duty_cycle = fan.get_fan_duty_cycle()
            for i in range (FAN_LEVEL_1, FAN_LEVEL_3 + 1):
                if self.init_duty_cycle == fan_speed_policy[i][0]:
                    fan_policy_state = i
                else:
                    continue
            logging.debug("- [Init]: fan_policy_state = %d, get duty_cycle = %d", fan_policy_state, self.init_duty_cycle)
            # Fan duty_cycle is not in FAN_LEVEL_1~3 case
            if fan_policy_state == LEVEL_FAN_INIT:
                if (self.init_duty_cycle > fan_speed_policy[FAN_LEVEL_2][0] and
                    self.init_duty_cycle < fan_speed_policy[FAN_LEVEL_3][0]):
                    fan_policy_state =  FAN_LEVEL_2
                else:
                    fan_policy_state =  FAN_LEVEL_1

                self.set_fan_duty_cycle(fan_policy_state, fan_speed_policy[fan_policy_state][0])

            return

        self.ori_duty_cycle = fan.get_fan_duty_cycle()
        self.new_duty_cycle = 0

        board_thermal_val_total  = 0  #SMB TMP75 (0X48) + UDB TMP75 (0X49) + UDB TMP422(0X48) + LDB TMP422(0X4d)
        board_thermal_th4_tmp422 = 0  #TH4 TMP422(0X4c)
        thermal_fan_policy_state = LEVEL_FAN_INIT
        cpu_fan_policy_state     = LEVEL_FAN_INIT
        mac_fan_policy_state     = LEVEL_FAN_INIT

        #1 Check fan: Unpresent or fan_fault status
        fan_fail = 0
        for i in range (fan.FAN_NUM_1_IDX, fan.FAN_NUM_ON_MAIN_BROAD+1):
            if fan.get_fan_present(i) == 0:
                fan_fail = 1
                logging.debug('- fan_%d absent, set duty_cycle to 100%', i)
            elif fan.get_fan_fault(i) == 1:
                fan_fail = 1
                logging.debug('- fan_%d fail, set duty_cycle to 100%', i)
            else:
                if fan_fail == 1:
                    continue

        ori_state     = fan_policy_state
        current_state = fan_policy_state

        if fan_fail == 1:
            if ori_state == FAN_LEVEL_2 or ori_state == FAN_LEVEL_3:
                current_state = FAN_LEVEL_3
            elif ori_state == FAN_LEVEL_1:
                current_state = FAN_LEVEL_2

            if current_state != ori_state:
                fan_policy_state = current_state
                self.new_duty_cycle = fan_speed_policy[fan_policy_state][0]

            if self.new_duty_cycle != self.ori_duty_cycle:
                self.set_fan_duty_cycle(fan_policy_state, fan_speed_policy[fan_policy_state][0])
                return True

        #2-1 Board Sensors get value:
        for i in range (thermal.THERMAL_NUM_1_IDX, thermal.THERMAL_NUM_5_IDX+1):
            board_thermal_val[i-1] = thermal._get_thermal_val(i)
            if i == thermal.THERMAL_NUM_5_IDX:
                board_thermal_th4_tmp422 = board_thermal_val[i-1]
            else:
                board_thermal_val_total += board_thermal_val[i-1]
        #2-2 CPU Sensors get value:
        for i in range (thermal.THERMAL_NUM_1_IDX, thermal.THERMAL_NUM_CPU_CORE+1):
            cpucore_thermal_val[i-1] = thermal._get_thermal_val(i + thermal.THERMAL_NUM_BD_SENSOR)

        #3-1 Decide the board thermal policy:
        if board_thermal_val_total <= thermal_spec["mid_to_min_temp"][0] and board_thermal_th4_tmp422 <= thermal_spec["mid_to_min_temp"][1]:
            thermal_fan_policy_state = FAN_LEVEL_1

        if board_thermal_val_total >= thermal_spec["min_to_mid_temp"][0] or board_thermal_th4_tmp422 >= thermal_spec["min_to_mid_temp"][1]:
            thermal_fan_policy_state = FAN_LEVEL_2

        #3-2 Decide the CPU thermal policy:
        for i in range (thermal.THERMAL_NUM_1_IDX, thermal.THERMAL_NUM_CPU_CORE+1):

            if cpucore_thermal_val[i-1] < thermal_spec["cpu_temp"][0]:      #Case of duty_cycle = 50%

                if cpu_fan_policy_state <= FAN_LEVEL_1:
                    # One of cpu-core is level_3, policy is level_3
                    cpu_fan_policy_state = FAN_LEVEL_1

            elif cpucore_thermal_val[i-1] >= thermal_spec["cpu_temp"][0] and cpucore_thermal_val[i-1] < thermal_spec["cpu_temp"][1]:

                if cpu_fan_policy_state <= FAN_LEVEL_3:
                    cpu_fan_policy_state = FAN_LEVEL_3                     #Case of duty_cycle = 100%

            elif cpucore_thermal_val[i-1] >= thermal_spec["cpu_temp"][1] :  #Case of shutdown

                logging.debug('CPU core%d, temperature is %d. Warning!!! Temperature is over %d', i-1, cpucore_thermal_val[i-1]/1000, thermal_spec["cpu_temp"][1]/1000)
                cpu_fan_policy_state = POLICY_NEED_SHUTDOWN

        #4 Condition of change fan speed by sensors policy:
        if ori_state == FAN_LEVEL_3:
            if cpu_fan_policy_state == POLICY_NEED_SHUTDOWN: # or mac shutdown
                # Need to implement Shutdown!!!!!!!!!!!!!
                print("shutdown happen!!")
                return False

            elif cpu_fan_policy_state == FAN_LEVEL_3: # Or MAC is level_3!!!!!!!!!!!!
                current_state = FAN_LEVEL_3

            else:
                current_state = FAN_LEVEL_2

        elif ori_state == FAN_LEVEL_2:
            if cpu_fan_policy_state == POLICY_NEED_SHUTDOWN: # or mac shutdown
                current_state = FAN_LEVEL_3

            elif cpu_fan_policy_state == FAN_LEVEL_3: # Or MAC is level_3!!!!!!!!!!!!
                current_state = FAN_LEVEL_3

            elif thermal_fan_policy_state == FAN_LEVEL_1:
                current_state = FAN_LEVEL_1

            else:
                current_state = FAN_LEVEL_2

        elif ori_state == FAN_LEVEL_1:
            if cpu_fan_policy_state == POLICY_NEED_SHUTDOWN: # or mac shutdown
                current_state = FAN_LEVEL_2

            elif cpu_fan_policy_state == FAN_LEVEL_3: # Or MAC is level_3!!!!!!!!!!!!
                current_state = FAN_LEVEL_2

            elif thermal_fan_policy_state == FAN_LEVEL_2:
                current_state = FAN_LEVEL_2

            else:
                current_state = FAN_LEVEL_1

        #4 Setting new duty-cyle:
        if current_state != ori_state:
            fan_policy_state = current_state

            self.new_duty_cycle = fan_speed_policy[fan_policy_state][0]

            if self.new_duty_cycle != self.ori_duty_cycle:
                self.set_fan_duty_cycle(fan_policy_state, fan_speed_policy[fan_policy_state][0])
                return True

            if self.new_duty_cycle == 0 :
                self.set_fan_duty_cycle(FAN_LEVEL_3, fan_speed_policy[FAN_LEVEL_3][0])

        return True

def signal_handler(sig, frame):
    global exit_by_sigterm
    if sig == signal.SIGTERM:
        print("Caught SIGTERM - exiting...")
        exit_by_sigterm = 1
    else:
        pass

def main(argv):
    log_file = '%s.log' % FUNCTION_NAME
    log_level = logging.INFO
    global test_temp
    global exit_by_sigterm
    signal.signal(signal.SIGTERM, signal_handler)
    if len(sys.argv) != 1:
        try:
            opts, args = getopt.getopt(argv,'hdl:',['lfile='])
        except getopt.GetoptError:
            print('Usage: %s [-d] [-l <log_file>]' % sys.argv[0])
            return 0
        for opt, arg in opts:
            if opt == '-h':
                print('Usage: %s [-d] [-l <log_file>]' % sys.argv[0])
                return 0
            elif opt in ('-d', '--debug'):
                log_level = logging.DEBUG
            elif opt in ('-l', '--lfile'):
                log_file = arg

    monitor = device_monitor(log_file, log_level)
    # Loop forever, doing something useful hopefully:
    while True:
        monitor.manage_fans()
        time.sleep(5)
        if exit_by_sigterm == 1:
            break

if __name__ == '__main__':
    main(sys.argv[1:])
