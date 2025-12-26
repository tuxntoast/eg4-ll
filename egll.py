# -*- coding: utf-8 -*-

# Notes
# Added by https://github.com/tuxntoast

# from batters import Protection
from battery import Battery, Cell
from utils import logger
from struct import unpack_from
from time import sleep
from pprint import pformat
import serial, struct, sys
import utils, datetime

#    Author: Pfitz
#    Date: 04 Jul 2025
#    Features:
#     UI reporting for all BMS in Communication chain
#     Multi BMS communication Chain Support
#     Cell Voltage Implemented
#     Hardware Name / Version / Serial Implemented
#     Error / Warn / Protection Implemented
#     SoH / SoC State Implemented
#     Temp Implemented
#     Balancing Support
#     Battery Voltage / Current
#     Support for 12v/24v/48v BMS

# Battery Tested on:
# 2x Eg4 LL 12v 400 AH
# Venus OS v3.60 running on Cerbo GX - dbus-serialbattery v2.0.20250518dev
# One RS232 Cable to USB is needed to connect Cerbo GX to the master BMS
# A Cat5/Cat6 cable can be used to connected the Master BMS RS485 secondary port to the
# first port of the BMS below it. BMS units can be "Daisy Chained" until your full bank is connected
# Update BATTERY_ADDRESSES in config.ini to define the BMS ID in your communication chain
# Example: BATTERY_ADDRESSES = 0x10, 0x01 would be to conect and report on BMS ID 16 and 1
#
# The master unit or first unit should have a Dip Switch ID set to 16 or 64 depending on your unit and version
# All other BMS in the communication chain should have a Dip switch setting of 1 - 15 or 1 - 63 depending on your units

class EG4_LL(Battery):
    def __init__(self, port, baud, address):

        super(EG4_LL, self).__init__(port, baud, address)
        self.cell_min_voltage = 0
        self.cell_max_voltage = None
        self.has_settings = 0
        self.reset_soc = 0
        self.soc_to_set = None
        self.type = self.BATTERYTYPE
        self.runtime = 0  # TROUBLESHOOTING for no reply errors

    statuslogger = False
    # When BMS returns a Warning/Error/Protection Alarm, Print that Error code
    # and BMS Stats to Driver Log file. This would help a user figure out what might be happening
    protectionLogger = True
    debug = False

    battery_stats = {}
    serialTimeout = 2

    BATTERYTYPE = "EG4-LL"

    hwCommandRoot = b"\x03\x00\x69\x00\x17"
    cellCommandRoot = b"\x03\x00\x00\x00\x27"
    bmsConfigCommandRoot = b"\x03\x00\x2D\x00\x5B"

    def unique_identifier(self):
        return self.serial_number

    def custom_name(self):
        self.custom_name = self.BATTERYTYPE+"_ID:"+str(self.Id)
        return self.custom_name

    def open_serial(self):
        ser = serial.Serial(self.port,
            baudrate=self.baud_rate,
            timeout=self.serialTimeout,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS)
        if ser.isOpen() == True:
            return ser
        else:
            return False

    def test_connection(self):
        try:
            self.battery_stats = {}
            self.Id = int.from_bytes(self.address, "big")
            self.ser = self.open_serial()
            BMS_list = self.discovery_pack(self.Id)
            if BMS_list is False:
                return False
            else:
                logger.error(f"Connected to BMS ID: {pformat(BMS_list)}")
                self.poll_interval = (((self.serialTimeout)*1000)*3)
                self.custom_field = self.BATTERYTYPE+"_ID:"+str(self.Id)
                cell_poll = self.read_battery_bank()
                self.cell_count = int(self.battery_stats[self.Id]["cell_count"])
                if cell_poll is not False:
                    self.status_logger()
                    return True
                else:
                    return False
        except Exception:
            (
                exception_type,
                exception_object,
                exception_traceback,
            ) = sys.exc_info()
            file = exception_traceback.tb_frame.f_code.co_filename
            line = exception_traceback.tb_lineno
            logger.error(
                f"Exception occurred: {repr(exception_object)} of type {exception_type} in {file} line #{line}"
            )

    def read_battery_bank(self):
        cell_reply = self.read_cell_details(self.Id)
        if cell_reply is not False:
            if self.Id in self.battery_stats:
                self.battery_stats[self.Id] = { **self.battery_stats[self.Id], **cell_reply }
            else:
                hw_reply = self.read_hw_details(self.Id)
                if hw_reply is not False:
                    self.cell_count = int(cell_reply["cell_count"])
                    self.min_battery_voltage = float(utils.MIN_CELL_VOLTAGE * self.cell_count)
                    self.max_battery_voltage = float(utils.MAX_CELL_VOLTAGE * self.cell_count)
                    self.capacity = cell_reply["capacity"]
                    self.battery_stats[self.Id] = { **cell_reply, **hw_reply }
                    self.read_bms_config()
                else:
                    return False
        else:
            return False
        result = self.reportBatteryBank()
        if self.statuslogger is True:
            self.status_logger()
        if result is True:
            return True
        return False

    def get_settings(self):
        result = self.read_battery_bank()
        if result is not True:
            return False
        self.status_logger()
        return True

    def refresh_data(self):
        # This will be called for every iteration
        result = self.read_battery_bank()
        if result is False:
            return False
        return True

    def read_gen_data(self):
        result = self.read_battery_bank()
        if result is False:
            return False
        return True

    def discovery_pack(self, Id):
        bmsChain = {}
        command = self.eg4CommandGen((Id.to_bytes(1, 'big') + self.hwCommandRoot))
        reply = self.read_eg4ll_command(command)
        if reply is not False:
            serial = (reply[33:48].decode("utf-8")+str(Id))
            bmsChain.update({Id: serial})
            self.serial_number = serial
            return bmsChain
        else:
            return False


    def read_hw_details(self, id):
        battery = {}
        command = self.eg4CommandGen((id.to_bytes(1, 'big') + self.hwCommandRoot))
        result = self.read_eg4ll_command(command)
        if result is False:
            return False

        battery.update({"hw_make" : result[2:25].decode("utf-8")})
        battery.update({"hw_version" : result[27:33].decode("utf-8")})
        battery.update({"hw_serial" : (result[33:48].decode("utf-8")+str(id))})
        battery.update({"hwLastPoll" : (datetime.datetime.now())})
        self.serial_number = battery["hw_serial"]
        self.version = battery["hw_make"]
        self.hardware_version = battery["hw_version"]

        return battery

    def read_cell_details(self, id):

        battery = {}
        command = self.eg4CommandGen((id.to_bytes(1, 'big') + self.cellCommandRoot))
        packet = self.read_eg4ll_command(command)
        if packet is False:
            return False
        battery.update({"voltage" : int.from_bytes(packet[3:5], "big") / 100})
        battery.update({"current" : int.from_bytes(packet[5:7], "big", signed=True) / 100})
        battery.update({"capacity_remain" : int.from_bytes(packet[45:47], "big")})
        battery.update({"capacity" : int.from_bytes(packet[65:69], "big") / 3600 / 1000})
        battery.update({"max_battery_charge_current" : int.from_bytes(packet[47:49], "big")})
        battery.update({"soc" : int.from_bytes(packet[51:53], "big")})
        battery.update({"soh" : int.from_bytes(packet[49:51], "big")})
        battery.update({"cycles" : int.from_bytes(packet[61:65], "big")})
        battery.update({"temp1" : int.from_bytes(packet[69:70], "big", signed=True)})
        battery.update({"temp2" : int.from_bytes(packet[70:71], "big", signed=True)})
        battery.update({"temp3" : int.from_bytes(packet[71:72], "big", signed=True)})
        battery.update({"temp4" : int.from_bytes(packet[72:73], "big", signed=True)})
        battery.update({"temperature_mos" : int.from_bytes(packet[39:41], "big", signed=True)})
        battery.update({"cell_count" : int.from_bytes(packet[75:77], "big")})
        battery.update({"status_hex" : packet[54:56].hex().upper()})
        battery.update({"warning_hex" : packet[55:57].hex().upper()})
        battery.update({"protection_hex" : packet[57:59].hex().upper()})
        battery.update({"error_hex" : packet[59:61].hex().upper()})
        battery.update({"heater_hex" : packet[53:54].hex().upper()})
        battery.update({"cellLastPoll" : (datetime.datetime.now())})
        startByte = 7
        endByte = 9
        cellId = 1
        cellVoltageList = []
        cellVoltageSum = 0
        while cellId <= battery["cell_count"]:
            cellNum = "cell"+str(cellId)
            cellVolt = int.from_bytes(packet[startByte:endByte], "big")/1000
            battery.update({cellNum : cellVolt})
            cellVoltageSum += float(battery[cellNum])
            cellVoltageList.append(battery.get(cellNum))
            startByte += 2
            endByte += 2
            cellId += 1
        battery.update({"cell_voltage" : round(cellVoltageSum, 3)})
        battery.update({"cell_max" : max(cellVoltageList)})
        battery.update({"cell_min" : min(cellVoltageList)})

        protection_status = self.lookup_protection(battery["protection_hex"])
        battery.update({"protection" : protection_status})
        warning_status = self.lookup_warning(battery["warning_hex"])
        battery.update({"warning" : warning_status})
        error_status = self.lookup_error(battery["error_hex"])
        battery.update({"error" : error_status})
        stats_status = self.lookup_status(battery["status_hex"])
        battery.update({"status" : stats_status})
        heater_status = self.lookup_heater(battery["heater_hex"])
        battery.update({"heater_state" : heater_status})
        balancing_code = self.balancingStat(battery["cell_min"], battery["cell_max"], battery["cell_voltage"])
        battery.update({"balancing_code" : balancing_code})
        if balancing_code == 2:
            battery.update({"balancing_text" : "Finished"})
        elif balancing_code == 1:
            battery.update({"balancing_text" : "Balancing"})
        elif balancing_code == 0:
            battery.update({"balancing_text" : "Off"})
        else:
            battery.update({"balancing_text" : "UNKNOWN"})

        return battery

    def reportBatteryBank(self):
        self.voltage = self.battery_stats[self.Id]["cell_voltage"]
        self.current = self.battery_stats[self.Id]["current"]
        self.capacity_remain = self.battery_stats[self.Id]["capacity_remain"]
        self.soc = self.battery_stats[self.Id]["soc"]
        self.soh = self.battery_stats[self.Id]["soh"]
        self.cycles = self.battery_stats[self.Id]["cycles"]
        self.temperature_1 = self.battery_stats[self.Id]["temp1"]
        self.temperature_2 = self.battery_stats[self.Id]["temp2"]
        if not (self.temperature_3 == 0) and (self.temperature_4 == 0):
            self.temperature_3 = self.battery_stats[self.Id]["temp3"]
            self.temperature_4 = self.battery_stats[self.Id]["temp4"]
        self.temperature_mos = self.battery_stats[self.Id]["temperature_mos"]
        self.cell_min_voltage = self.battery_stats[self.Id]["cell_min"]
        self.cell_max_voltage = self.battery_stats[self.Id]["cell_max"]
        self.temp_min = min(self.temperature_1, self.temperature_2)
        self.temp_max = max(self.temperature_1, self.temperature_2)
        self.cells = [Cell(True) for _ in range(0, self.cell_count)]
        for i, cell in enumerate(self.cells):
            self.cells[int(i)].voltage = self.battery_stats[self.Id]['cell'+str(i+1)]

        return True

    def balance_status(self):
        logger.info(f"== BMS ID-{self.Id} ===")
        logger.info("   == BMS Data ==")
        logger.info("       Voltage: "
            + "%.3fv" % self.battery_stats[self.Id]["cell_voltage"]
            + " | Current: "
            + str(self.battery_stats[self.Id]["current"])
            + "A"
        )
        logger.info(f"       Capacity Left: {self.battery_stats[self.Id]["capacity_remain"]} of {self.battery_stats[self.Id]["capacity"]} AH")
        logger.info(f"       Balancing State: {self.battery_stats[self.Id]['balancing_text']}")
        logger.info(f"       State: {self.battery_stats[self.Id]['status']}")
        logger.info(f"       Last Update: {self.battery_stats[self.Id]['cellLastPoll']}")
        logger.info("     == Cell Stats ==")
        cellId = 1
        while cellId <= self.battery_stats[self.Id]['cell_count']:
            logger.info(f"       Cell {str(cellId)} Voltage: {self.battery_stats[self.Id]['cell'+str(cellId)]}")
            cellId += 1
        logger.info(f"       Cell Max/Min/Diff: ({self.battery_stats[self.Id]['cell_max']}/{self.battery_stats[self.Id]['cell_min']}/{round((self.battery_stats[self.Id]['cell_max'] - self.battery_stats[self.Id]['cell_min']), 3)})v")
        return True

    def bms_stats(self):
        logger.info("== Pack Details =====")
        logger.info(f'Serial Number: {self.battery_stats[self.Id]["hw_serial"]}')
        logger.info(f"  == BMS ID-{self.Id} ===")
        logger.info(f"    == Temp ==")
        logger.info(f"      Temp 1: {self.battery_stats[self.Id]["temp1"]}c | Temp 2: {self.battery_stats[self.Id]["temp2"]}c | Temp Mos: {self.battery_stats[self.Id]["temperature_mos"]}c")
        logger.info("     == BMS Data ==")
        logger.info("       Voltage: "
            + "%.3fv" % self.battery_stats[self.Id]["cell_voltage"]
            + " | Current: "
            + str(self.battery_stats[self.Id]["current"])
            + "A"
        )
        logger.info(f"       Capacity Left: {self.battery_stats[self.Id]["capacity_remain"]} of {self.battery_stats[self.Id]["capacity"]} AH")
        logger.info(f"       Balancing State: {self.battery_stats[self.Id]['balancing_text']}")
        logger.info(f"       State: {self.battery_stats[self.Id]['status']}")
        logger.info(f"       Last Update: {self.battery_stats[self.Id]['cellLastPoll']}")
        logger.info("     == Cell Stats ==")
        cellId = 1
        while cellId <= self.battery_stats[self.Id]['cell_count']:
            logger.info(f"       Cell {str(cellId)} Voltage: {self.battery_stats[self.Id]['cell'+str(cellId)]}")
            cellId += 1
        logger.info(f"       Cell Max/Min/Diff: ({self.battery_stats[self.Id]['cell_max']}/{self.battery_stats[self.Id]['cell_min']}/{round((self.battery_stats[self.Id]['cell_max'] - self.battery_stats[self.Id]['cell_min']), 3)})v")
        return True

    def status_logger(self):
        logger.info("===== HW Info =====")
        logger.info(f'Battery Make/Model: {self.battery_stats[self.Id]["hw_make"]}')
        logger.info(f'Hardware Version: {self.battery_stats[self.Id]["hw_version"]}')
        logger.info(f'Serial Number: {self.battery_stats[self.Id]["hw_serial"]}')
        logger.info("===== Temp =====")
        logger.info(f"Temp 1: {self.temperature_1}c | Temp 2: {self.temperature_2}c | Temp Mos: {self.temperature_mos}c")
        logger.info(f"Temp Max: {self.temp_max} | Temp Min: {self.temp_min}")
        logger.info(f"Heater {self.Id} Status: {self.battery_stats[self.Id]["heater_state"]}")
        logger.info("===== BMS Data =====")
        logger.info("Voltage: "
            + "%.3fv" % self.voltage
            + " | Current: "
            + str(self.current)
            + "A"
        )
        logger.info(f"Capacity Left: {self.capacity_remain} of {self.capacity} AH")
        logger.info(f"SoC: {self.soc}%")
        logger.info(f"SoH: {self.soh}% | Cycle Count: {self.cycles}")
        logger.info(f"Balancing State: {self.battery_stats[self.Id]['balancing_text']}")
        logger.info("===== Warning/Alarms =====")
        logger.info(f"  {self.battery_stats[self.Id]["warning"]}")
        logger.info(f"  {self.battery_stats[self.Id]["protection"]}")
        logger.info(f"  {self.battery_stats[self.Id]["error"]}")
        logger.info("===== Pack Details =====")
        logger.info(f"  === BMS ID-{self.Id} ===")
        logger.info(f"  State: {self.battery_stats[self.Id]['status']}")
        logger.info(f"  Pack Balancing: {self.battery_stats[self.Id]['balancing_text']}")
        logger.info(f"  Last Update: {self.battery_stats[self.Id]['cellLastPoll']}")
        logger.info(f"  Pack Voltage: {round((self.battery_stats[self.Id]['cell_voltage']),3)}v | Pack Current: {round((self.battery_stats[self.Id]['current']),2)}a")
        logger.info("    = Cell Stats =")
        cellId = 1
        while cellId <= self.battery_stats[self.Id]['cell_count']:
            logger.info(f"  Cell {str(cellId)} Voltage: {self.battery_stats[self.Id]['cell'+str(cellId)]}")
            cellId += 1
        logger.info(f"  Cell Max/Min/Diff: ({self.battery_stats[self.Id]['cell_max']}/{self.battery_stats[self.Id]['cell_min']}/{round((self.battery_stats[self.Id]['cell_max'] - self.battery_stats[self.Id]['cell_min']), 3)})v")
        return True

    def lookup_warning(self, code):
        warning_alarm = ""
        if code == "0000":
            warning_alarm += "No Warnings - "+code
        elif code == "0800":
            warning_alarm += "Warning: "+code+" - Charge Under Temp"
            self.temp_high_charge = 1
        else:
            warning_alarm += "Warning: "+code+" - UNKNOWN"
        if self.protectionLogger is True:
            if code != "0000":
                logger.error(f"Warning Alarm:{warning_alarm}")
                self.bms_stats()
        return warning_alarm

    def lookup_protection(self, code):
        protection_alarm = ""
        if code == "0000":
            protection_alarm += "No Protection Events - "+code
            self.charge_fet = True
            self.discharge_fet = True
        else:
            protection_alarm += "Protection UNKNOWN: "+code
        if self.protectionLogger is True:
            if code != "0000":
                logger.error(f"Protection Alarm: {protection_alarm}")
                self.bms_stats()
        return protection_alarm

    def lookup_error(self, code):
        error_alarm = ""
        if code == "0000":
            error_alarm = f"No Errors - "+code
        elif code == "8000":
            error_alarm = f"Error: "+code+" - Temperature Error"
            logger.error(f"BMS Error: {error_alarm}")
            self.bms_stats()
        else:
            error_alarm = "UNKNOWN: "+code
        if self.protectionLogger is True:
            if code != "0000":
                logger.error(f"BMS Error: {error_alarm}")
                self.bms_stats()
        return error_alarm

    def lookup_status(self, status_hex):
        status_code = ""
        if status_hex == "0000":
            status_code = "Active/Standby"
        elif status_hex == "0100":
            status_code = "Active/Charging"
        elif status_hex == "0200":
            status_code = "Active/Discharging"
        elif status_hex == "0008":
            status_code = "Inactive/Protect"
            logger.error(f"BMS Status: {status_code}")
            self.balance_status()
        elif status_hex == "0800":
            status_code = "Active/Charging Limit"
            logger.error(f"BMS Status: {status_code}")
            self.balance_status()
        else:
            status_code = "UNKNOWN - "+status_hex
            logger.error(f"BMS Status: {status_code}")
            self.balance_status()
        return status_code

    def lookup_heater(self, heater_status):
        if heater_status == "00":
            heater_state = False
        elif heater_status == "80":
            heater_state = True
        return heater_state

    def get_balancing(self):
        balacing_state = self.balancingStat(self.battery_stats[self.Id]["cell_min"], self.battery_stats[self.Id]["cell_max"], self.battery_stats[self.Id]["cell_voltage"])
        return balacing_state

    def balancingStat(self, cellMin, cellMax, cellSum):
        balancer_current_delta = .040
        pack_maxVoltage = 14.2
        balancer_voltage = 3.40
        if cellSum >= pack_maxVoltage:
            if (round((cellMax - cellMin), 3) <= balancer_current_delta) and (cellMin >= balancer_voltage): # Fully Charged
                if self.balacing_state == 1: # State change from active to balance complete
                    logger.info(f"Finished Balancing on BMS {self.Id}")
                    self.balance_status()
                self.balacing_state = 2
                self.balance_fet = False
        elif (round((cellMax - cellMin), 3) >= balancer_current_delta) and (cellMin >= balancer_voltage): # Min Volt limit reached, cell diff enabling balancer
            self.balacing_state = 1
            self.balance_fet = True
        elif (round((cellMax - cellMin), 3) < balancer_current_delta) and (cellMin >= balancer_voltage): # Not fully Charged, but still below balance limit
            self.balacing_state = 0
            self.balance_fet = True
        else: # too low voltage,
            self.balacing_state = 0
            self.balance_fet = False
        return self.balacing_state

    def get_max_temperature(self):
        if not (self.temperature_3 == 0) and (self.temperature_4 == 0):
            temp_max = max(self.temperature_1, self.temperature_2, self.temperature_3, self.temperature_4)
        else:
            temp_max = max(self.temperature_1, self.temperature_2)
        return temp_max

    def get_min_temperature(self):
        if not (self.temperature_3 == 0) and (self.temperature_4 == 0):
            temp_min = min(self.temperature_1, self.temperature_2, self.temperature_3, self.temperature_4)
        else:
            temp_min = min(self.temperature_1, self.temperature_2)
        return temp_min

    def read_bms_config(self):
        logger.info("Executed read_bms_config function... function needs to be written")
        command = self.eg4CommandGen((self.Id.to_bytes(1, 'big') + self.bmsConfigCommandRoot))
        packet = self.read_eg4ll_command(command)
        if packet is False:
            return False
        BMSConf = {}
        BMSConf.update({"Balance_Volt" : int.from_bytes(packet[25:27], 'big') / 1000})
        BMSConf.update({"Balance_Volt_Diff" : int.from_bytes(packet[27:29], 'big') / 1000})
        BMSConf.update({"Low_Cap_Warn" : int.from_bytes(packet[29:31], 'big')})
        BMSConf.update({"Cell_UV_Warn" : int.from_bytes(packet[35:37], 'big') / 1000})
        BMSConf.update({"Cell_OV_Warn" : int.from_bytes(packet[47:49], 'big') / 1000})
        BMSConf.update({"Cell_UV_Protect" : int.from_bytes(packet[37:39], 'big') / 1000})
        BMSConf.update({"Cell_OV_Protect" : int.from_bytes(packet[49:51], 'big') / 1000})
        BMSConf.update({"Cell_UV_Release" : int.from_bytes(packet[39:41], 'big') / 1000})
        BMSConf.update({"Cell_OV_Release" : int.from_bytes(packet[51:53], 'big') / 1000})
        BMSConf.update({"Pack_UV_Warn" : int.from_bytes(packet[41:43], 'big') / 100})
        BMSConf.update({"Pack_OV_Warn" : int.from_bytes(packet[53:55], 'big') / 100})
        BMSConf.update({"Pack_UV_Protect" : int.from_bytes(packet[43:45], 'big') / 100})
        BMSConf.update({"Pack_OV_Protect" : int.from_bytes(packet[55:57], 'big') / 100})
        BMSConf.update({"Pack_UV_Release" : int.from_bytes(packet[45:47], 'big') / 100})
        BMSConf.update({"Pack_OV_Release" : int.from_bytes(packet[57:59], 'big') / 100})
        logger.info(f"disCharge Release : {packet[69:71].hex().upper()} :: {int.from_bytes(packet[69:71], 'big')}")
        logger.info(f"disCharge Release : {packet[71:73].hex().upper()} :: {int.from_bytes(packet[71:73], 'big')}")
        logger.info(f"disCharge OC1 Protect : {packet[73:75].hex().upper()} :: {int.from_bytes(packet[73:75], 'big') / 100}")
        logger.info(f"disCharge OC1 Delay : {packet[83:85].hex().upper()} :: {int.from_bytes(packet[83:85], 'big')}")
        logger.info(f"disCharge OC1 Protect : {packet[75:77].hex().upper()} :: {int.from_bytes(packet[75:77], 'big') / 100}")
        logger.info(f"disCharge OC1 Delay : {packet[87:89].hex().upper()} :: {int.from_bytes(packet[87:89], 'big')}")
        logger.info(f"disCharge OC2 Protect : {packet[79:81].hex().upper()} :: {int.from_bytes(packet[79:81], 'big') / 100}")
        logger.info(f"disCharge OC2 Delay : {packet[85:87].hex().upper()} :: {int.from_bytes(packet[85:87], 'big')}")
        logger.info(f"disCharge OC2 Protect : {packet[81:83].hex().upper()} :: {int.from_bytes(packet[81:83], 'big') / 100}")
        logger.info(f"disCharge OC2 Delay : {packet[89:91].hex().upper()} :: {int.from_bytes(packet[89:91], 'big')}")
        logger.info(f"Load Short Current : {packet[77:79].hex().upper()} :: {int.from_bytes(packet[77:79], 'big') / 100}")
        logger.info(f"UNKNOWN : {packet[93:95].hex().upper()} :: {int.from_bytes(packet[93:95], 'big', signed=True)}")
        logger.info(f"UNKNOWN : {packet[95:97].hex().upper()} :: {int.from_bytes(packet[95:97], 'big', signed=True)}")
        logger.info(f"UNKNOWN : {packet[97:99].hex().upper()} :: {int.from_bytes(packet[97:99], 'big', signed=True)}")
        logger.info(f"UNKNOWN : {packet[99:101].hex().upper()} :: {int.from_bytes(packet[99:101], 'big', signed=True)}")
        logger.info(f"UNKNOWN : {packet[101:103].hex().upper()} :: {int.from_bytes(packet[101:103], 'big', signed=True)}")
        logger.info(f"UNKNOWN : {packet[103:105].hex().upper()} :: {int.from_bytes(packet[103:105], 'big', signed=True)}")
        logger.info(f"UNKNOWN : {packet[105:107].hex().upper()} :: {int.from_bytes(packet[105:107], 'big', signed=True)}")
        logger.info(f"UNKNOWN : {packet[107:109].hex().upper()} :: {int.from_bytes(packet[107:109], 'big', signed=True)}")
        logger.info(f"UNKNOWN : {packet[109:111].hex().upper()} :: {int.from_bytes(packet[109:111], 'big', signed=True)}")
        logger.info(f"UNKNOWN : {packet[111:113].hex().upper()} :: {int.from_bytes(packet[111:113], 'big', signed=True)}")
        logger.info(f"UNKNOWN : {packet[113:115].hex().upper()} :: {int.from_bytes(packet[113:115], 'big', signed=True)}")
        logger.info(f"UNKNOWN : {packet[115:117].hex().upper()} :: {int.from_bytes(packet[115:117], 'big', signed=True)}")
        logger.info(f"UNKNOWN : {packet[117:119].hex().upper()} :: {int.from_bytes(packet[117:119], 'big', signed=True)}")
        logger.info(f"UNKNOWN : {packet[119:121].hex().upper()} :: {int.from_bytes(packet[119:121], 'big', signed=True)}")
        logger.info(f"UNKNOWN : {packet[121:123].hex().upper()} :: {int.from_bytes(packet[121:123], 'big', signed=True)}")

        return True

    def eg4CommandGen(self, data: bytes):
        # CRC-16-ModBus Algorithm
        poly = 0xA001
        crc = 0xFFFF
        for b in data:
            crc ^= (0xFF & b)
            for _ in range(0, 8):
                if (crc & 0x0001):
                    crc = ((crc >> 1) & 0xFFFF) ^ poly
                else:
                    crc = ((crc >> 1) & 0xFFFF)
        reverseHex = struct.pack('<H', crc)
        command = data + reverseHex
        return command

    # Read data from previously opened serial port
    def read_eg4ll_command(self, command):
        attemptCount = 0
        try:
            CommandHex = command.hex(":").upper()
            bmsId = int(CommandHex[0:2], 16)
            cmdId = CommandHex[9:11]

            if cmdId == "69":
                commandString = "Hardware"
                reply_length = 51
            elif cmdId == "00":
                commandString = "Cell"
                reply_length = 83
            elif cmdId == "2D":
                commandString = "Config"
                reply_length = 187
            else:
                commandString = "UNKNOWN"

            if self.ser.isOpen() == True:
                while attemptCount <= 3:
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                    self.ser.write(command)
                    pollCount = 0
                    toread = self.ser.inWaiting()
                    attemptCount += 1
                    while toread < reply_length:
                        sleep(0.035)
                        toread = self.ser.inWaiting()
                        pollCount += 1
                        if toread == reply_length:
                            break
                        if pollCount > 50:
                            if attemptCount == 3 and cmdId == "00":
                                logger.error(f'No Reply - BMS ID: {bmsId} Command: {commandString} - Attempt: {attemptCount}')
                                return False
                            elif cmdId == "69":
                                logger.error(f'No Reply - BMS ID: {bmsId} Command: {commandString}')
                                return False
                            elif cmdId != "00":
                                return False
                            else:
                                break
                    if toread == reply_length:
                        break
                res = self.ser.read(toread)
                data = bytearray(res)
            else:
                logger.error(f'ERROR - Serial Port Not Open!')
                self.ser = self.open_serial()
                return False

            if toread == reply_length:
                return data
            else:
                logger.error(f'ERROR - Reply not meet expected length! BMS ID: {bmsId} Command: {commandString}')
                return False
        except serial.SerialException as e:
            logger.error(e)
            return False
