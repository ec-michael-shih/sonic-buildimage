#!/usr/bin/env python
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
# ------------------------------------------------------------------
# HISTORY:
#    mm/dd/yyyy (A.D.)
#    12/07/2022:Michael_Shih craete for as9736_64d
# ------------------------------------------------------------------

try:
    import logging
    import subprocess
    from sonic_platform.helper import APIHelper
except ImportError as e:
    raise ImportError('%s - required module not found' % str(e))

TARGET_SPEED_PATH = "/tmp/fan_target_speed"

class FanUtil(object):
    """Platform-specific FanUtil class"""

    FAN_NUM_ON_MAIN_BROAD = 4
    FAN_NUM_1_IDX = 1
    FAN_NUM_2_IDX = 2
    FAN_NUM_3_IDX = 3
    FAN_NUM_4_IDX = 4

    FAN_NODE_NUM_OF_MAP = 5
    FAN_NODE_FAULT_IDX_OF_MAP = 1
    FAN_NODE_DUTY_IDX_OF_MAP  = 2
    FAN_NODE_FRONT_SPD_OF_MAP = 3
    FAN_NODE_REAR_SPD_OF_MAP  = 4
    FAN_NODE_PRESENT_IDX_OF_MAP = 5

    BASE_VAL_PATH = '/sys/bus/i2c/devices/25-0033/{0}'

    """ Dictionary where
        key1 = fan id index (integer) starting from 1
        key2 = fan node index (interger) starting from 1
        value = path to fan device file (string)
    """
    _fan_device_path_mapping = {}

    _fan_device_node_mapping = {
           (FAN_NUM_1_IDX, FAN_NODE_FAULT_IDX_OF_MAP)  : 'fan1_fault',
           (FAN_NUM_1_IDX, FAN_NODE_DUTY_IDX_OF_MAP)   : 'fan1_duty_cycle_percentage',
           (FAN_NUM_1_IDX, FAN_NODE_FRONT_SPD_OF_MAP)  : 'fan1_front_speed_rpm',
           (FAN_NUM_1_IDX, FAN_NODE_REAR_SPD_OF_MAP)   : 'fan1_rear_speed_rpm',
           (FAN_NUM_1_IDX, FAN_NODE_PRESENT_IDX_OF_MAP): 'fan1_present',

           (FAN_NUM_2_IDX, FAN_NODE_FAULT_IDX_OF_MAP)  : 'fan2_fault',
           (FAN_NUM_2_IDX, FAN_NODE_DUTY_IDX_OF_MAP)   : 'fan2_duty_cycle_percentage',
           (FAN_NUM_2_IDX, FAN_NODE_FRONT_SPD_OF_MAP)  : 'fan2_front_speed_rpm',
           (FAN_NUM_2_IDX, FAN_NODE_REAR_SPD_OF_MAP)   : 'fan2_rear_speed_rpm',
           (FAN_NUM_2_IDX, FAN_NODE_PRESENT_IDX_OF_MAP): 'fan2_present',

           (FAN_NUM_3_IDX, FAN_NODE_FAULT_IDX_OF_MAP)  : 'fan3_fault',
           (FAN_NUM_3_IDX, FAN_NODE_DUTY_IDX_OF_MAP)   : 'fan3_duty_cycle_percentage',
           (FAN_NUM_3_IDX, FAN_NODE_FRONT_SPD_OF_MAP)  : 'fan3_front_speed_rpm',
           (FAN_NUM_3_IDX, FAN_NODE_REAR_SPD_OF_MAP)   : 'fan3_rear_speed_rpm',
           (FAN_NUM_3_IDX, FAN_NODE_PRESENT_IDX_OF_MAP): 'fan3_present',

           (FAN_NUM_4_IDX, FAN_NODE_FAULT_IDX_OF_MAP)  : 'fan4_fault',
           (FAN_NUM_4_IDX, FAN_NODE_DUTY_IDX_OF_MAP)   : 'fan4_duty_cycle_percentage',
           (FAN_NUM_4_IDX, FAN_NODE_FRONT_SPD_OF_MAP)  : 'fan4_front_speed_rpm',
           (FAN_NUM_4_IDX, FAN_NODE_REAR_SPD_OF_MAP)   : 'fan4_rear_speed_rpm',
           (FAN_NUM_4_IDX, FAN_NODE_PRESENT_IDX_OF_MAP): 'fan4_present',
           }

    #def _get_fan_device_node(self, fan_num, node_num):
    #    return self._fan_device_node_mapping[(fan_num, node_num)]

    def _get_fan_node_val(self, fan_num, node_num):
        if fan_num < self.FAN_NUM_1_IDX or fan_num > self.FAN_NUM_ON_MAIN_BROAD:
            logging.debug('GET. Parameter error. fan_num:%d', fan_num)
            return None

        if node_num < self.FAN_NODE_FAULT_IDX_OF_MAP or node_num > self.FAN_NODE_NUM_OF_MAP:
            logging.debug('GET. Parameter error. node_num:%d', node_num)
            return None

        device_path = self.get_fan_device_path(fan_num, node_num)

        try:
            val_file = open(device_path, 'r')
        except IOError as e:
            logging.error('GET. unable to open file: %s', str(e))
            return None

        content = val_file.readline().rstrip()

        if content == '':
            logging.debug('GET. content is NULL. device_path:%s', device_path)
            return None

        try:
            val_file.close()
        except IOError:
            logging.debug('GET. unable to close file. device_path:%s', device_path)
            return None

        return int(content)

    def _set_fan_node_val(self, fan_num, node_num, val):
        if fan_num < self.FAN_NUM_1_IDX or fan_num > self.FAN_NUM_ON_MAIN_BROAD:
            logging.debug('GET. Parameter error. fan_num:%d', fan_num)
            return None

        if node_num < self.FAN_NODE_FAULT_IDX_OF_MAP or node_num > self.FAN_NODE_NUM_OF_MAP:
            logging.debug('GET. Parameter error. node_num:%d', node_num)
            return None

        content = str(val)
        if content == '':
            logging.debug('GET. content is NULL. device_path:%s', device_path)
            return None

        device_path = self.get_fan_device_path(fan_num, node_num)
        try:
            val_file = open(device_path, 'w')
        except IOError as e:
            logging.error('GET. unable to open file: %s', str(e))
            return None

        val_file.write(content)

        try:
            val_file.close()
        except Exception:
            logging.debug('GET. unable to close file. device_path:%s', device_path)
            return None

        return True

    def __init__(self):
        fan_path = self.BASE_VAL_PATH

        for fan_num in range(self.FAN_NUM_1_IDX, self.FAN_NUM_ON_MAIN_BROAD+1):
            for node_num in range(self.FAN_NODE_FAULT_IDX_OF_MAP, self.FAN_NODE_NUM_OF_MAP+1):
                self._fan_device_path_mapping[(fan_num, node_num)] = fan_path.format(
                   self._fan_device_node_mapping[(fan_num, node_num)])

    def get_num_fans(self):
        return self.FAN_NUM_ON_MAIN_BROAD

    def get_idx_fan_start(self):
        return self.FAN_NUM_1_IDX

    def get_num_nodes(self):
        return self.FAN_NODE_NUM_OF_MAP

    def get_idx_node_start(self):
        return self.FAN_NODE_FAULT_IDX_OF_MAP

    def get_size_node_map(self):
        return len(self._fan_device_node_mapping)

    def get_size_path_map(self):
        return len(self._fan_device_path_mapping)

    def get_fan_device_path(self, fan_num, node_num):
        return self._fan_device_path_mapping[(fan_num, node_num)]

    def get_fan_fault(self, fan_num):
        return self._get_fan_node_val(fan_num, self.FAN_NODE_FAULT_IDX_OF_MAP)

    def get_fan_present(self, fan_num):
        return self._get_fan_node_val(fan_num, self.FAN_NODE_PRESENT_IDX_OF_MAP)

    def get_fan_duty_cycle(self):
        duty_path = self._fan_device_path_mapping[(self.FAN_NUM_1_IDX, self.FAN_NODE_DUTY_IDX_OF_MAP)]
        try:
            val_file = open(duty_path)
        except IOError as e:
            print("Error: unable to open file: %s" % str(e))
            return False

        content = val_file.readline().rstrip()
        val_file.close()

        return int(content)

    def set_fan_duty_cycle(self, val):
        for fan_num in range(1, self.FAN_NUM_ON_MAIN_BROAD + 1):
            duty_path = self._fan_device_path_mapping[(fan_num, self.FAN_NODE_DUTY_IDX_OF_MAP)]
            try:
                fan_file = open(duty_path, 'r+')
            except IOError as e:
                print("Error: unable to open file: %s" % str(e))
                return False

            fan_file.write(str(val))
            fan_file.close()

        # Update set_fan_speed to host and pmon
        APIHelper.write_txt_file(self, TARGET_SPEED_PATH, int(val))
        subprocess.getstatusoutput("docker exec pmon bash -c 'echo {} > {}'".format(val, TARGET_SPEED_PATH))

        return True

    def get_fan_front_speed(self, fan_num):
        return self._get_fan_node_val(fan_num, self.FAN_NODE_FRONT_SPD_OF_MAP)

    def get_fan_rear_speed(self, fan_num):
        return self._get_fan_node_val(fan_num, self.FAN_NODE_REAR_SPD_OF_MAP)

    def get_fan_status(self, fan_num):
        if fan_num < self.FAN_NUM_1_IDX or fan_num > self.FAN_NUM_ON_MAIN_BROAD:
            logging.debug('GET. Parameter error. fan_num, %d', fan_num)
            return None

        if self.get_fan_fault(fan_num) is not None and self.get_fan_fault(fan_num) > 0:
            logging.debug('GET. FAN fault. fan_num, %d', fan_num)
            return False

        return True
