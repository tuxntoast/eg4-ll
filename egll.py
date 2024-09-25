# -*- coding: utf-8 -*-

# Notes
# Added by https://github.com/tuxntoast

from battery import Battery, Cell

# from batters import Protection
from utils import logger
from struct import unpack_from
from time import sleep
from pprint import pformat
import serial, struct, sys
import utils, datetime

#    Author: Pfitz
#    Date: 24 Sept 2024
#    Active Dev:
#     - Starting to add support for BMS channing / more then one battery unit
#       Tasks:
#            - Passing all cell voltage to OS, but as different pack
#                Issue: DVCC will add all cell to find charge voltage, this should be sum of all cell in pack
#    Features:
#     Cell Voltage Implemented
#     Hardware Name / Version / Serial Implemented
#     Error / Warn / Protection Implemented
#     SoH / SoC State Implemented
#     Temp Implemented
#     Balancing Support
#     Battery Voltage / Current
#     Multi BMS communication Chain Support
#     Support for 12v/24v/48v BMS
#     Support for Generation 1 & 2 -  EG4 Server Rack Batteries
#        - Has not been tested on the rack mount units.

# Battery Tested on:
# 2x Eg4 LL 12v 400 AH
# One RS232 Cable to USB is needed to connect Cerbo GX to the master BMS
# A Cat5/Cat6 cable can be used to connected the Master BMS RS485 secondary port to the
# first port of the BMS below it. BMS units can be "Daisy Chained" until your full bank is connected
#
# The master unit or first unit should have a Dip Switch ID set to 16 or 64 depending on your unit and version
# All other BMS in the communication chain should have a Dip switch setting of 1 - 15 or 1 - 63 depending on your units

class EG4_LL(Battery):
    def __init__(self, port, baud, address):

        super(EG4_LL, self).__init__(port, baud, address)
        self.cell_min_voltage = 0
        self.cell_max_voltage = None
        self.type = self.BATTERYTYPE
        self.has_settings = 0
        self.reset_soc = 0
        self.soc_to_set = None
        self.runtime = 0  # TROUBLESHOOTING for no reply errors

    statuslogger = False

    ##batteryPackId = utils.EG4_LL_BATTERY_PACK_IDS
    batteryPackId = [ 16, 1 ]
    batteryMasterId = int(batteryPackId[0])
    battery_stats = {}
    serialTimeout = 2

    BATTERYTYPE = "EG4 LL"
    balacing_text = "UNKNOWN"

    hwCommandRoot = b"\x03\x00\x69\x00\x17"
    cellCommandRoot = b"\x03\x00\x00\x00\x27"
    bmsConfigCommandRoot = b"\x03\x00\x2D\x00\x5B"

    def unique_identifier(self):
        return self.serial_number 

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
        # call a function that will connect to the battery, send a command and retrieve the result.
        # The result or call should be unique to this BMS. Battery name or version, etc.
        # Return True if success, False for failure
        try:
            self.battery_stats = {}
            self.ser = self.open_serial()
            BMS_list = self.discovery_pack()
            if len(BMS_list) > 0 and self.batteryMasterId in BMS_list:
                self.serial_number = BMS_list[self.batteryMasterId]
                self.battery_stats[self.batteryMasterId] = self.read_cell_details(self.batteryMasterId)
                if self.battery_stats[self.batteryMasterId] is not False:
                    reply = self.rollupBatteryBank(self.battery_stats)
                    if reply != "Failed":
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
        for id in self.batteryPackId:
            cell_reply = self.read_cell_details(id)
            if cell_reply is not False:
                if id in self.battery_stats:
                    self.battery_stats[id] = { **self.battery_stats[id], **cell_reply }
                else:
                    hw_reply = self.read_hw_details(id)
                    if hw_reply is not False and cell_reply is not False:
                        self.battery_stats[id] = { **cell_reply, **hw_reply }
        result = self.rollupBatteryBank(self.battery_stats)
        if self.statuslogger is True:
            self.status_logger()
        return True

    def get_settings(self):
        # After successful  connection get_settings will be call to set up the battery.
        # Return True if success, False for failure
        id = 1
        battery_stats = {}
        retry = True
        self.poll_interval = ( (len(self.batteryPackId) * self.serialTimeout)*1000)
        for id in self.batteryPackId:
            cell_reply = self.read_cell_details(id)
            hw_reply = self.read_hw_details(id)
            if hw_reply is not False and cell_reply is not False:
                self.battery_stats[id] = { **cell_reply, **hw_reply }
        result = self.rollupBatteryBank(self.battery_stats)
        self.status_logger()
        return True

    def refresh_data(self):
        # call all functions that will refresh the battery data.
        # This will be called for every iteration (1 second)
        # Return True if success, False for failure
        result = self.read_battery_bank()
        if result is False:
            return False
        return True

    def read_gen_data(self):
        result = self.read_battery_bank()
        if result is False:
            return False
        return True

    def discovery_pack(self):
        bmsChain = {}
        for Id in self.batteryPackId:
            command = self.eg4CommandGen((Id.to_bytes(1, 'big') + self.hwCommandRoot))
            reply = self.read_eg4ll_command(command)
            if reply is not False:
                serial = (reply[33:48].decode("utf-8")+"_"+str(Id))
                bmsChain.update({Id : serial})
        logger.info(f"Connected to BMS ID's: {pformat(bmsChain)}")
        return bmsChain

    def read_hw_details(self, id):
        battery = {}
        command = self.eg4CommandGen((id.to_bytes(1, 'big') + self.hwCommandRoot))
        result = self.read_eg4ll_command(command)
        if result is False:
            return False

        battery.update({"hw_make" : result[2:25].decode("utf-8")})
        battery.update({"hw_version" : result[27:33].decode("utf-8")})
        battery.update({"hw_serial" : (result[33:48].decode("utf-8")+"_"+str(id))})
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
        battery.update({"temp1" : int.from_bytes(packet[39:41], "big", signed=True)})
        battery.update({"temp2" : int.from_bytes(packet[69:70], "big", signed=True)})
        battery.update({"temp_mos" : int.from_bytes(packet[70:71], "big", signed=True)})
        battery.update({"temp_max" : max(battery["temp1"], battery["temp2"])})
        battery.update({"temp_min" : min(battery["temp1"], battery["temp2"])})
        battery.update({"cell_count" : int.from_bytes(packet[75:77], "big")})
        battery.update({"status_hex" : packet[54:55].hex().upper()})
        battery.update({"warning_hex" : packet[55:57].hex().upper()})
        battery.update({"protection_hex" : packet[57:59].hex().upper()})
        battery.update({"error_hex" : packet[59:61].hex().upper()})
        battery.update({"heater_status" : packet[53:54].hex().upper()})
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
        battery.update({"cell_voltage" : cellVoltageSum})
        battery.update({"cell_max" : max(cellVoltageList)})
        battery.update({"cell_min" : min(cellVoltageList)})
        balancing_code = self.status_balancing(battery["cell_max"], battery["cell_min"], "code")
        balancing_text = self.status_balancing(battery["cell_max"], battery["cell_min"], "text")
        battery.update({"balancing_code" : balancing_code})
        battery.update({"balancing_text" : balancing_text})
        return battery

    def rollupBatteryBank(self, batteryBankStats):

        if self.battery_stats is False or self.battery_stats[self.batteryMasterId] is False:
            return "Failed"

        #logger.info(f"batteryBankStats: {pformat(batteryBankStats)}")
        self.voltage = self.battery_stats[self.batteryMasterId]["cell_voltage"]
        self.current = self.battery_stats[self.batteryMasterId]["current"]
        self.capacity_remain = self.battery_stats[self.batteryMasterId]["capacity_remain"]
        self.capacity = self.battery_stats[self.batteryMasterId]["capacity"]
        self.soc = self.battery_stats[self.batteryMasterId]["soc"]
        self.soh = self.battery_stats[self.batteryMasterId]["soh"]
        self.cycles = self.battery_stats[self.batteryMasterId]["cycles"]
        self.temp1 = self.battery_stats[self.batteryMasterId]["temp1"]
        self.temp2 = self.battery_stats[self.batteryMasterId]["temp2"]
        self.temp_mos = self.battery_stats[self.batteryMasterId]["temp_mos"]
        self.cell_min_voltage = self.battery_stats[self.batteryMasterId]["cell_min"]
        self.cell_max_voltage = self.battery_stats[self.batteryMasterId]["cell_max"]
        self.lookup_protection(self.battery_stats)
        self.lookup_warning(self.battery_stats)

        if len(self.battery_stats) > 1:
            for bmsId in self.battery_stats:
                if bmsId != self.batteryMasterId:
                    if self.battery_stats[bmsId] is not False:
                        self.voltage = round(((self.voltage + self.battery_stats[bmsId]["cell_voltage"]) / 2), 3)
                        self.current = round((self.current + self.battery_stats[bmsId]["current"]), 3)
                        self.capacity_remain = (self.capacity_remain + self.battery_stats[bmsId]["capacity_remain"])
                        self.capacity = (self.capacity + self.battery_stats[bmsId]["capacity"])
                        self.soc = (self.soc + self.battery_stats[bmsId]["soc"]) / 2
                        self.soh = (self.soh + self.battery_stats[bmsId]["soh"]) / 2
                        if self.battery_stats[bmsId]["cycles"] > self.cycles:
                            self.cycles = self.battery_stats[bmsId]["cycles"]
                        if self.battery_stats[bmsId]["temp1"] > self.temp1:
                            self.temp1 = self.battery_stats[bmsId]["temp1"]
                        if self.battery_stats[bmsId]["temp2"] > self.temp2:
                            self.temp2 = self.battery_stats[bmsId]["temp2"]
                        if self.battery_stats[bmsId]["temp_mos"] > self.temp_mos:
                            self.temp_mos = self.battery_stats[bmsId]["temp_mos"]
                        if self.battery_stats[bmsId]["cell_max"] > self.cell_max_voltage:
                            self.cell_max_voltage = self.battery_stats[bmsId]["cell_max"]
                        if self.battery_stats[bmsId]["cell_min"] > self.cell_min_voltage:
                            self.cell_min_voltage = self.battery_stats[bmsId]["cell_min"]

        self.temp_max = max(self.temp1, self.temp2)
        self.temp_min = min(self.temp1, self.temp2)

        self.cell_count = batteryBankStats[self.batteryMasterId]["cell_count"]
        self.min_battery_voltage = float(utils.MIN_CELL_VOLTAGE * self.cell_count)
        self.max_battery_voltage = float(utils.MAX_CELL_VOLTAGE * self.cell_count)
        self.cells = [Cell(True) for _ in range(0, self.cell_count)]
        for i, cell in enumerate(self.cells):
            self.cells[int(i)].voltage = self.battery_stats[self.batteryMasterId]['cell'+str(i+1)]

        return True

    def status_logger(self):
        if self.battery_stats is not False or self.battery_stats[self.batteryMasterId] is not False:
            logger.info("===== HW Info =====")
            logger.info(f'Battery Make/Model: {self.battery_stats[self.batteryMasterId]["hw_make"]}')
            logger.info(f'Hardware Version: {self.battery_stats[self.batteryMasterId]["hw_version"]}')
            for bmsId in self.battery_stats:
                if self.battery_stats[bmsId] is not False:
                    logger.info(f'Serial Number: {self.battery_stats[bmsId]["hw_serial"]}')
            logger.info("===== Temp =====")
            logger.info(f"Temp 1: {self.temp1}c | Temp 2: {self.temp2}c | Temp Mos: {self.temp_mos}c")
            logger.info(f"Temp Max: {self.temp_max} | Temp Min: {self.temp_min}")
            for bmsId in self.battery_stats:
                if self.battery_stats[bmsId] is not False:
                    logger.info(f"Heater {bmsId} Status: {self.lookup_heater(self.battery_stats[bmsId]['heater_status'])}")
            logger.info("===== DVCC State =====")
            logger.info(f"DVCC Charger Mode: {self.charge_mode}")
            logger.info(f"DVCC Charge Voltage: {self.control_voltage}v")
            logger.info(
                f"Charge Current: {self.control_charge_current} | Discharge Current: {self.control_discharge_current}"
            )
            logger.info(
                f"Charge Limit: {self.charge_limitation} | Discharge Limit: {self.discharge_limitation}"
            )
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
            logger.info(f"Balancing State: {self.balacing_text}")
            logger.info("===== Warning/Alarms =====")
            logger.info(f"  {self.lookup_warning(self.battery_stats)}")
            logger.info(f"  {self.lookup_protection(self.battery_stats)}")
            logger.info(f"  {self.lookup_error(self.battery_stats)}")
            logger.info("===== Pack Details =====")
            for bmsId in self.battery_stats:
                if self.battery_stats[bmsId] is not False:
                    cellId = 1
                    logger.info(f"  === BMS ID-{bmsId} ===")
                    logger.info(f"  State: {self.lookup_status(self.battery_stats[bmsId]['status_hex'])}")
                    logger.info(f"  Pack Balancing: {self.battery_stats[bmsId]['balancing_text']}")
                    logger.info(f"  Last Update: {self.battery_stats[bmsId]['cellLastPoll']}")
                    logger.info(f"  Pack Voltage: {round((self.battery_stats[bmsId]['cell_voltage']),3)}v | Pack Current: {round((self.battery_stats[bmsId]['current']),2)}a")
                    logger.info("    = Cell Stats =")
                    while cellId <= self.battery_stats[bmsId]['cell_count']:
                        logger.info(f"  Cell {str(cellId)} Voltage: {self.battery_stats[bmsId]['cell'+str(cellId)]}")
                        cellId += 1
                    logger.info(f"  Cell Max/Min/Diff: ({self.battery_stats[bmsId]['cell_max']}/{self.battery_stats[bmsId]['cell_min']}/{round((self.battery_stats[bmsId]['cell_max'] - self.battery_stats[bmsId]['cell_min']), 3)})v")
        return True

    def lookup_warning(self, batteryBankStats):

        unique_codes = []
        for bmsId in batteryBankStats:
            if batteryBankStats[bmsId] is not False:
                if batteryBankStats[bmsId]["warning_hex"] not in unique_codes:
                    unique_codes.append(batteryBankStats[bmsId]["warning_hex"])
        warning_alarm = ""
        for code in unique_codes:
            if code == "0000":
                warning_alarm += "No Warnings - "+code
            elif code == "0001":
                warning_alarm += "Warning: "+code+" - Pack Over Voltage"
                self.voltage_high = 1
            elif code == "0002":
                warning_alarm += "Warning: "+code+" - Cell Over Voltage"
                self.voltage_cell_high = 1
            elif code == "0004":
                warning_alarm += "Warning: "+code+" - Pack Under Voltage"
                self.voltage_low = 1
            elif code == "0008":
                warning_alarm += "Warning: "+code+" - Cell Under Voltage"
                self.voltage_cell_low = 1
            elif code == "0010":
                warning_alarm += "Warning: "+code+" - Charge Over Current"
                self.current_over = 1
            elif code == "0020":
                warning_alarm += "Warning: "+code+" - Discharge Over Current"
                self.current_over = 1
            elif code == "0040":
                warning_alarm += "Warning: "+code+" - Ambient High Temp"
                self.temp_high_internal = 1
            elif code == "0080":
                warning_alarm += "Warning: "+code+" - Mosfets High Temp"
                self.temp_high_internal = 1
            elif code == "0100":
                warning_alarm += "Warning: "+code+" - Charge Over Temp"
                self.temp_high_charge = 1
            elif code == "0200":
                warning_alarm += "Warning: "+code+" - Discharge Over Temp"
                self.temp_high_discharge = 1
            elif code == "0400":
                warning_alarm += "Warning: "+code+" - Charge Under Temp"
                self.temp_low_charge = 1
            elif code == "1000":
                warning_alarm += "Warning: "+code+" - Low Capacity"
                self.soc_low = 1
            elif code == "2000":
                warning_alarm += "Warning: "+code+" - Float Stoped"
            elif code == "4000":
                warning_alarm += "Warning: "+code+" - UNKNOWN"
                self.internal_failure = 1
            else:
                warning_alarm += "Warning: "+code+" - UNKNOWN"

        return warning_alarm

    def lookup_protection(self, batteryBankStats):
        unique_codes = []
        for bmsId in batteryBankStats:
            if batteryBankStats[bmsId] is not False:
                if batteryBankStats[bmsId]["protection_hex"] not in unique_codes:
                    unique_codes.append(batteryBankStats[bmsId]["protection_hex"])
        protection_alarm = ""
        for code in unique_codes:
            if code == "0000":
                protection_alarm += "No Protection Events - "+code
            elif code == "0001":
                protection_alarm += "Protection: "+code+" - Pack Over Voltage"
                self.voltage_high = 2
            elif code == "0002":
                protection_alarm += "Protection: "+code+" - Cell Over Voltage"
                self.voltage_cell_high = 2
            elif code == "0004":
                protection_alarm += "Protection: "+code+" - Pack Under Voltage"
                self.voltage_low = 2
            elif code == "0008":
                protection_alarm += "Protection: "+code+" - Cell Under Voltage"
                self.voltage_cell_low = 2
            elif code == "0010":
                protection_alarm += "Protection: "+code+" - Charge Over Current"
                self.current_over = 2
            elif code == "0020":
                protection_alarm += "Protection: "+code+" - Discharge Over Current"
                self.current_over = 2
            elif code == "0040":
                protection_alarm += "Protection: "+code+" - High Ambient Temp"
                self.temp_high_internal = 2
            elif code == "0080":
                protection_alarm += "Protection: "+code+" - Mosfets High Temp"
                self.temp_high_internal = 2
            elif code == "0100":
                protection_alarm += "Protection: "+code+" - Charge Over Temp"
                self.temp_high_charge = 2
            elif code == "0200":
                protection_alarm += "Protection: "+code+" - Discharge Over Temp"
                self.temp_high_discharge = 2
            elif code == "0400":
                protection_alarm += "Protection: "+code+" - Charge Under Temp"
                self.temp_low_charge = 2
            elif code == "0800":
                protection_alarm += "Protection: "+code+" - Discharge Under Temp"
                self.temp_low_charge = 2
            elif code == "1000":
                protection_alarm += "Protection: "+code+" - Low Capacity"
                self.soc_low = 2
            elif code == "2000":
                protection_alarm += "Protection: "+code+" - Discharge SC"
            else:
                protection_alarm += "UNKNOWN: "+code
        return protection_alarm

    def lookup_error(self, batteryBankStats):
        unique_codes = []
        for bmsId in batteryBankStats:
            if batteryBankStats[bmsId] is not False:
                if batteryBankStats[bmsId]["error_hex"] not in unique_codes:
                    unique_codes.append(batteryBankStats[bmsId]["error_hex"])
        error_alarm = ""
        for code in unique_codes:
            if code == "0000":
                error_alarm = f"No Errors - "+code
            elif code == "0001":
                error_alarm = f"Error: "+code+" - Voltage Error"
            elif code == "0002":
                error_alarm = f"Error: "+code+" - Temperature Error"
            elif code == "0004":
                error_alarm = f"Error: "+code+" - Current Flow Error"
            elif code == "0010":
                error_alarm = f"Error: "+code+" - Cell Unbalanced"
            else:
                error_alarm = "UNKNOWN: "+code
        return error_alarm

    def lookup_status(self, status_hex):
        if status_hex == "00":
            status_code = "Standby"
        elif status_hex == "01":
            status_code = "Charging"
        elif status_hex == "02":
            status_code = "Discharging"
        elif status_hex == "04":
            status_code = "Protect"
        elif status_hex == "08":
            status_code = "Charging Limit"
        return status_code

    def lookup_heater(self, heater_status):
        if heater_status == "00":
            heater_state = False
        elif heater_status == "80":
            heater_state = True
        return heater_state

    def get_balancing(self):
        balancingSummery = []
        bmsId = 1
        for bmsId in self.battery_stats:
            if self.battery_stats[bmsId] is not False:
                stateCode = self.status_balancing(self.battery_stats[bmsId]['cell_max'], self.battery_stats[bmsId]['cell_min'], "code")
                balancingSummery.append(stateCode)
            else:
                return 0
        balancingFinished = all(ele in balancingSummery for ele in [2])
        if balancingFinished is True:
            balacing_state = 2
            self.balacing_text = "Finished"
        elif 1 in balancingSummery:
            balacing_state = 1
            self.balacing_text = "Balancing"
        else:
            balacing_state = 0
            self.balacing_text = "Off"
        return balacing_state

    def status_balancing(self, cell_max, cell_min, reply):
        balancer_current_delta = .40
        balancer_voltage = 3.40
        if (cell_max > balancer_voltage) and (round((cell_max - cell_min), 3) <= balancer_current_delta):
            balacing_state = 2
            balacing_text = "Finished"
        elif (cell_max - cell_min) >= balancer_current_delta:
            if cell_max >= balancer_voltage:
                balacing_state = 1
                balacing_text = "Balancing"
        else:
            balacing_state = 0
            balacing_text = "Off"

        if reply == "text":
            return balacing_text
        elif reply == "code":
            return balacing_state
        else:
            return False

    def get_max_temp(self):
        self.temp1 = self.battery_stats[self.batteryMasterId]["temp1"]
        self.temp2 = self.battery_stats[self.batteryMasterId]["temp2"]
        temp_max = max(self.temp1, self.temp2)
        return temp_max

    def get_min_temp(self):
        self.temp1 = self.battery_stats[self.batteryMasterId]["temp1"]
        self.temp2 = self.battery_stats[self.batteryMasterId]["temp2"]
        temp_min = min(self.temp1, self.temp2)
        return temp_min

    def read_bms_config(self):
        logger.info("Executed read_bms_config function... function needs to be written")
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
                res = self.ser.read(toread)
                data = bytearray(res)
            else:
                logger.error(f'ERROR - Serial Port Not Open!')
                self.ser = self.open_serial()
                return False

            if toread == reply_length:
                return data
            else:
                return False
                logger.error(f'ERROR - Reply Not meet exspected length!')
        except serial.SerialException as e:
            logger.error(e)
            return False
