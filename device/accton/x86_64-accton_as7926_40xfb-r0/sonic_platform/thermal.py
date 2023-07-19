#############################################################################
# Edgecore
#
# Thermal contains an implementation of SONiC Platform Base API and
# provides the thermal device status which are available in the platform
#
#############################################################################

import os
import os.path

try:
    from sonic_platform_base.thermal_base import ThermalBase
    from .helper import APIHelper
except ImportError as e:
    raise ImportError(str(e) + "- required module not found")

THERMAL_NAME_LIST = ["Temp sensor 1", "Temp sensor 2", "Temp sensor 3",
                     "Temp sensor 4", "Temp sensor 5", "Temp sensor 6",
                     "Temp sensor 7", "Temp sensor 8", "Temp sensor 9",
                     "Temp sensor 10", "CPU Package Temp",
                     "CPU Core 0 Temp", "CPU Core 1 Temp", "CPU Core 2 Temp",
                     "CPU Core 3 Temp", "CPU Core 4 Temp", "CPU Core 5 Temp",
                     "CPU Core 6 Temp", "CPU Core 7 Temp",]

PSU_THERMAL_NAME_LIST = ["PSU-1 temp sensor 1", "PSU-2 temp sensor 1"]



class Thermal(ThermalBase):
    """Platform-specific Thermal class"""

    SYSFS_PATH = "/sys/devices/platform/as7926_40xfb_thermal/"
    SYSFS_PATH_PSU = "/sys/devices/platform/as7926_40xfb_psu/"

    def __init__(self, thermal_index=0, is_psu=False, psu_index=0):
        self._api_helper = APIHelper()
        self.index = thermal_index
        self.is_psu = is_psu
        self.psu_index = psu_index

        if self.is_psu:
            self.hwmon_path = self.SYSFS_PATH_PSU
        else:
            self.hwmon_path = self.SYSFS_PATH

        self.ss_index = self.index + 1

        self.HIGH_THRESHOLD_FIELD = 'high_threshold'
        self.HIGH_CRIT_THRESHOLD_FIELD = 'high_critical_threshold'

        self.default_threshold = {
            THERMAL_NAME_LIST[0] : {
                self.HIGH_THRESHOLD_FIELD : '84.0',
                self.HIGH_CRIT_THRESHOLD_FIELD : '87.0'
            },
            THERMAL_NAME_LIST[1] : {
                self.HIGH_THRESHOLD_FIELD : '110.0',
                self.HIGH_CRIT_THRESHOLD_FIELD : '115.0'
            },
            THERMAL_NAME_LIST[2] : {
                self.HIGH_THRESHOLD_FIELD : '110.0',
                self.HIGH_CRIT_THRESHOLD_FIELD : '115.0'
            },
            THERMAL_NAME_LIST[3] : {
                self.HIGH_THRESHOLD_FIELD : '70.0',
                self.HIGH_CRIT_THRESHOLD_FIELD : '73.0'
            },
            THERMAL_NAME_LIST[4] : {
                self.HIGH_THRESHOLD_FIELD : '72.0',
                self.HIGH_CRIT_THRESHOLD_FIELD : '75.0'
            },
            THERMAL_NAME_LIST[5] : {
                self.HIGH_THRESHOLD_FIELD : '73.0',
                self.HIGH_CRIT_THRESHOLD_FIELD : '76.0'
            },
            THERMAL_NAME_LIST[6] : {
                self.HIGH_THRESHOLD_FIELD : '70.0',
                self.HIGH_CRIT_THRESHOLD_FIELD : '73.0'
            },
            THERMAL_NAME_LIST[7] : {
                self.HIGH_THRESHOLD_FIELD : '62.0',
                self.HIGH_CRIT_THRESHOLD_FIELD : '65.0'
            },
            THERMAL_NAME_LIST[8] : {
                self.HIGH_THRESHOLD_FIELD : '84.0',
                self.HIGH_CRIT_THRESHOLD_FIELD : '87.0'
            },
            THERMAL_NAME_LIST[9] : {
                self.HIGH_THRESHOLD_FIELD : '76.0',
                self.HIGH_CRIT_THRESHOLD_FIELD : '79.0'
            },
            THERMAL_NAME_LIST[10] : {
                self.HIGH_THRESHOLD_FIELD : '82.0',
                self.HIGH_CRIT_THRESHOLD_FIELD : '104.0'
            },
            THERMAL_NAME_LIST[11] : {
                self.HIGH_THRESHOLD_FIELD : '82.0',
                self.HIGH_CRIT_THRESHOLD_FIELD : '104.0'
            },
            THERMAL_NAME_LIST[12] : {
                self.HIGH_THRESHOLD_FIELD : '82.0',
                self.HIGH_CRIT_THRESHOLD_FIELD : '104.0'
            },
            THERMAL_NAME_LIST[13] : {
                self.HIGH_THRESHOLD_FIELD : '82.0',
                self.HIGH_CRIT_THRESHOLD_FIELD : '104.0'
            },
            THERMAL_NAME_LIST[14] : {
                self.HIGH_THRESHOLD_FIELD : '82.0',
                self.HIGH_CRIT_THRESHOLD_FIELD : '104.0'
            },
            THERMAL_NAME_LIST[15] : {
                self.HIGH_THRESHOLD_FIELD : '82.0',
                self.HIGH_CRIT_THRESHOLD_FIELD : '104.0'
            },
            THERMAL_NAME_LIST[16] : {
                self.HIGH_THRESHOLD_FIELD : '82.0',
                self.HIGH_CRIT_THRESHOLD_FIELD : '104.0'
            },
            THERMAL_NAME_LIST[17] : {
                self.HIGH_THRESHOLD_FIELD : '82.0',
                self.HIGH_CRIT_THRESHOLD_FIELD : '104.0'
            },
            THERMAL_NAME_LIST[18] : {
                self.HIGH_THRESHOLD_FIELD : '82.0',
                self.HIGH_CRIT_THRESHOLD_FIELD : '104.0'
            },
            PSU_THERMAL_NAME_LIST[0] : {
                self.HIGH_THRESHOLD_FIELD : '62.0',
                self.HIGH_CRIT_THRESHOLD_FIELD : '67.0'
            },
            PSU_THERMAL_NAME_LIST[1] : {
                self.HIGH_THRESHOLD_FIELD : '62.0',
                self.HIGH_CRIT_THRESHOLD_FIELD : '67.0'
            }
        }

    def __get_temp(self, temp_file):
        temp_file_path = os.path.join(self.hwmon_path, temp_file)

        raw_temp = self._api_helper.read_txt_file(temp_file_path)
        if raw_temp is not None:
            return float(raw_temp)/1000
        else:
            return 0

    def get_temperature(self):
        """
        Retrieves current temperature reading from thermal
        Returns:
            A float number of current temperature in Celsius up to nearest thousandth
            of one degree Celsius, e.g. 30.125
        """
        if not self.is_psu:
            temp_file = "temp{}_input".format(self.ss_index)
        else:
            temp_file = "psu{}_temp{}_input".format(self.psu_index+1, self.ss_index)

        return self.__get_temp(temp_file)

    def get_high_threshold(self):
        """
        Retrieves the high threshold temperature of thermal
        Returns:
            A float number, the high threshold temperature of thermal in Celsius
            up to nearest thousandth of one degree Celsius, e.g. 30.125
        """
        default_value = self.default_threshold[self.get_name()][self.HIGH_THRESHOLD_FIELD]
        return float(default_value)

    def get_high_critical_threshold(self):
        """
        Retrieves the high critical threshold temperature of thermal

        Returns:
            A float number, the high critical threshold temperature of thermal in Celsius
            up to nearest thousandth of one degree Celsius, e.g. 30.125
        """
        default_value = self.default_threshold[self.get_name()][self.HIGH_CRIT_THRESHOLD_FIELD]
        return float(default_value)

    def set_high_threshold(self, temperature):
        """
        Sets the high threshold temperature of thermal
        Args :
            temperature: A float number up to nearest thousandth of one degree Celsius,
            e.g. 30.125
        Returns:
            A boolean, True if threshold is set successfully, False if not
        """
        return True

    def get_name(self):
        """
        Retrieves the name of the thermal device
            Returns:
            string: The name of the thermal device
        """
        if self.is_psu:
            return PSU_THERMAL_NAME_LIST[self.psu_index]
        else:
            return THERMAL_NAME_LIST[self.index]

    def get_presence(self):
        """
        Retrieves the presence of the Thermal
        Returns:
            bool: True if Thermal is present, False if not
        """
        if self.is_psu:
            temp_file = self.hwmon_path + "psu{}_present".format(self.psu_index+1)
            val = self._api_helper.read_txt_file(temp_file)
            return int(val, 10) == 1

        temp_file = "temp{}_input".format(self.ss_index)
        temp_file_path = os.path.join(self.hwmon_path, temp_file)
        raw_txt = self._api_helper.read_txt_file(temp_file_path)
        if raw_txt is not None:
            return True
        else:
            return False

    def get_status(self):
        """
        Retrieves the operational status of the device
        Returns:
            A boolean value, True if device is operating properly, False if not
        """
        if self.is_psu:
            return self.get_presence()

        file_str = "temp{}_input".format(self.ss_index)
        file_path = os.path.join(self.hwmon_path, file_str)
        raw_txt = self._api_helper.read_txt_file(file_path)
        if raw_txt is None:
            return False
        else:
            return int(raw_txt) != 0

    def get_model(self):
        """
        Retrieves the model number (or part number) of the device
        Returns:
            string: Model/part number of device
        """

        return "N/A"

    def get_serial(self):
        """
        Retrieves the serial number of the device
        Returns:
            string: Serial number of device
        """
        return "N/A"

    def get_position_in_parent(self):
        """
        Retrieves 1-based relative physical position in parent device.
        If the agent cannot determine the parent-relative position
        for some reason, or if the associated value of
        entPhysicalContainedIn is'0', then the value '-1' is returned
        Returns:
            integer: The 1-based relative physical position in parent device
            or -1 if cannot determine the position
        """
        return self.index+1

    def is_replaceable(self):
        """
        Retrieves whether thermal module is replaceable
        Returns:
            A boolean value, True if replaceable, False if not
        """
        return False
