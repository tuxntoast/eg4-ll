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
from bms.EG4_ll_alarm_manager import EG4AlarmManager

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
# Venus OS v3.67 running on Cerbo GX - dbus-serialbattery v2.0.20250729
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
        self.data = {}   
        self.alarm_mgr = EG4AlarmManager(self.data, bms_stats_cb=self.bms_stats)

    # Print all the data collected from the BMS each polling cycles to the log
    # when set to true
    statuslogger = False
    # When BMS returns a Warning/Error/Protection Alarm, Print that Error code
    # and BMS Stats to Driver Log file. This would help a user figure out what might be happening
    LoadBMSSettings = True
    # When a Protection, Error, Status or Warning is found during a poll of the BMS, print that Error Code
    # to Log as INFO & print basic stats of what was found from that BMS at that same polling time.
    # **Caution** - Warning, Error, Protection, Status are not documented publicly what hex means what
    # under manual testing, BMS is inconsistent when and what data point and ove what time period get
    # them to fire off. When Daisy Chained, one BMS seems to report events.
    protectionLogger = True

    battery_stats = {}
    serialTimeout = 2

    BATTERYTYPE = "EG4-LL"

    hwCommandRoot = b"\x03\x00\x69\x00\x17"
    cellCommandRoot = b"\x03\x00\x00\x00\x27"
    bmsConfigCommandRoot = b"\x03\x00\x2D\x00\x5B"

    def unique_identifier(self):
        return self.serial_number

    def custom_name(self):
        self.custom_name = self.BATTERYTYPE+":"+str(self.Id)
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
            command = self.eg4CommandGen((self.Id.to_bytes(1, 'big') + self.hwCommandRoot))
            reply = self.read_eg4ll_command(command)
            if reply is False:
                return False
            else:
                serial = (reply[33:48].decode("utf-8")+str(self.Id))
                logger.error(f"Connected to BMS ID: {pformat(serial)}")
                self.serial_number = serial
                self.poll_interval = (((self.serialTimeout)*1000)*3)
                self.custom_field = self.BATTERYTYPE+"_ID:"+str(self.Id)
                cell_poll = self.read_battery_bank()
                if cell_poll is True:
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
            logger.error(f"Exception occurred: {repr(exception_object)} of type {exception_type} in {file} line #{line}")

    def read_battery_bank(self) -> bool:
        cell_reply = self.read_cell_details(self.Id)
        if not cell_reply:
            return False
        bank_stats = self.battery_stats.get(self.Id)
        first_run = bank_stats is None
        protection_status = self.lookup_protection(cell_reply)
        cell_reply["protection"] = protection_status
        warning_status = self.lookup_warning(cell_reply)
        cell_reply["warning"] = warning_status
        error_status = self.lookup_error(cell_reply)
        cell_reply["error"] = error_status
        bms_status = self.lookup_status(cell_reply)
        cell_reply["status"] = bms_status
        heater_status = self.lookup_heater(cell_reply)
        cell_reply["heater_state"] = heater_status
        cell_reply["BMS_Id"] = self.Id
        # First-time initialization → fetch static HW/BMS data once
        if first_run:
            if self.LoadBMSSettings is True:
                extra_reply = self.read_bms_config()
                if extra_reply is False:
                    extra_reply = self.read_hw_details(self.Id)

            else:
                extra_reply = self.read_hw_details(self.Id)
            if not extra_reply:
                return False
            else:
                bank_stats = {**extra_reply, **cell_reply}
                #self.alarm_mgr = EG4AlarmManager(bms_data=bank_stats, bms_stats_cb=self.bms_stats)
                self.alarm_mgr = EG4AlarmManager(self.data, bms_stats_cb=self.bms_stats)

            self.cell_count = int(cell_reply["cell_count"])
            self.min_battery_voltage = utils.MIN_CELL_VOLTAGE * cell_reply["cell_count"]
            self.max_battery_voltage = utils.MAX_CELL_VOLTAGE * cell_reply["cell_count"]
            self.capacity = bank_stats["capacity"]
            self.serial_number = bank_stats["hw_serial"]
            self.version = bank_stats["hw_make"]
            self.hardware_version = bank_stats["hw_version"]
        else:
            # Battery already initialized → only update cell data
            bank_stats.update(cell_reply)

        # ---- Balancing state ----
        balancing_map = {
            0: "Off",
            1: "Balancing",
            2: "Finished"}
        code = self.balancingStat(bank_stats)
        bank_stats["balancing_code"] = code
        bank_stats["balancing_text"] = balancing_map.get(code, "UNKNOWN")
        self.reportBatteryBank(bank_stats)

        # Evaluate alarms FIRST
        alarm_status = self.alarm_mgr.evaluate(new_data=bank_stats)
        # Only call bms_stats if something is not normal
        if alarm_status and any(v != 0 for v in alarm_status.values()):
            self.bms_stats(bank_stats, alarm_status)

        self.battery_stats[self.Id] = bank_stats
        if first_run and not getattr(self, "_initial_status_logged", False):
            self.status_logger(bank_stats)
            self._initial_status_logged = True
            return True
        elif any(bank_stats.get(k, "0000") != "0000" for k in ["protection_hex", "warning_hex", "error_hex"]):
            if self.protectionLogger is True:
                logger.error(f"**BMS Error, Protect or Warning Event Code Found In Polling Cycle**")
                self.bms_stats(bank_stats)
                return True
        elif self.statuslogger is True:
            self.status_logger(bank_stats)
        return True

    def get_settings(self):
        result = self.read_battery_bank()
        if result is not True:
            return False
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
        return battery

    def read_cell_details(self, id):
        command = self.eg4CommandGen(id.to_bytes(1, "big") + self.cellCommandRoot)
        packet = self.read_eg4ll_command(command)
        # ---- Parsing helpers ----
        u16 = lambda o: int.from_bytes(packet[o:o+2], "big")
        s16 = lambda o: int.from_bytes(packet[o:o+2], "big", signed=True)
        u32 = lambda o: int.from_bytes(packet[o:o+4], "big")
        s8  = lambda o: int.from_bytes(packet[o:o+1], "big", signed=True)
        CELL_START = 7
        MAX_CELLS = 16
        battery = {
            "voltage": u16(3) / 100,
            "current": s16(5) / 100,
            "capacity_remain": u16(45),
            "capacity": u32(65) / 3600 / 1000,
            "max_battery_charge_current": u16(47),
            "soc": u16(51),
            "soh": u16(49),
            "cycles": u32(61),
            "temperature_mos": s16(39),
            "cell_count": min(u16(75), MAX_CELLS),
            "status_hex": packet[54:56].hex().upper(),
            "warning_hex": packet[55:57].hex().upper(),
            "protection_hex": packet[57:59].hex().upper(),
            "error_hex": packet[59:61].hex().upper(),
            "heater_hex": packet[53:54].hex().upper(),
            "cellLastPoll": datetime.datetime.now(),
        }
        # ---- Cell voltages (0x0000 = not present) ----
        cells = []
        for i in range(MAX_CELLS):
            raw = u16(CELL_START + i * 2)
            if raw == 0:
                continue
            voltage = raw / 1000
            battery[f"cell{i+1}"] = voltage
            cells.append(voltage)
        battery["cell_count"] = len(cells)
        if cells:
            battery["cell_voltage"] = round(sum(cells), 3)
            battery["cell_max"] = max(cells)
            battery["cell_min"] = min(cells)
        # ---- Temperature handling (special rules) ----
        temp1 = s8(69)  # always valid
        temp2 = s8(70)  # always valid
        temp3 = s8(71)  # 0 = not present
        temp4 = s8(72)  # 0 = not present
        battery["temp1"] = temp1
        battery["temp2"] = temp2
        temps = [temp1, temp2]
        if temp3 != 0:
            battery["temp3"] = temp3
            temps.append(temp3)
        if temp4 != 0:
            battery["temp4"] = temp4
            temps.append(temp4)
        battery["temp_min"] = min(temps)
        battery["temp_max"] = max(temps)
        return battery

    def reportBatteryBank(self, packet):
        self.voltage = packet["cell_voltage"]
        self.current = packet["current"]
        self.capacity_remain = packet["capacity_remain"]
        self.soc = packet["soc"]
        self.soh = packet["soh"]
        self.cycles = packet["cycles"]
        self.temperature_1 = packet["temp1"]
        self.temperature_2 = packet["temp2"]
        if not (self.temperature_3 == 0) and (self.temperature_4 == 0):
            self.temperature_3 = packet["temp3"]
            self.temperature_4 = packet["temp4"]
        self.temperature_mos = packet["temperature_mos"]
        self.cell_min_voltage = packet["cell_min"]
        self.cell_max_voltage = packet["cell_max"]
        self.temp_min = min(self.temperature_1, self.temperature_2)
        self.temp_max = max(self.temperature_1, self.temperature_2)
        self.cells = [Cell(True) for _ in range(0, self.cell_count)]
        for i, cell in enumerate(self.cells):
            self.cells[int(i)].voltage = packet['cell'+str(i+1)]
        # Evaluate alarms
        #alarm_status = self.alarm_mgr.evaluate()
        return True

    def balance_status(self, packet):
        logger.info(f"== BMS ID-{packet["BMS_Id"]} ===")
        logger.info("   == BMS Data ==")
        logger.info("       Voltage: "
            + "%.3fv" % packet["cell_voltage"]
            + " | Current: "
            + str(packet["current"])
            + "A"
        )
        logger.info(f"       Balancing State: {packet["balancing_text"]}")
        logger.info(f"       State: {packet["status"]}")
        logger.info(f"       Last Update: {packet["cellLastPoll"]}")
        logger.info("     == Cell Stats ==")
        cellId = 1
        while cellId <= packet['cell_count']:
            logger.info(f"       Cell {str(cellId)} Voltage: {packet['cell'+str(cellId)]}")
            cellId += 1
        logger.info(f"       Cell Max/Min/Diff: ({packet["cell_max"]}/{packet["cell_min"]}/{round((packet["cell_max"] - packet["cell_min"]), 3)})v")
        return True

    def bms_stats(self, packet, alarm_msg=None):
        logger.info("== Pack Details =====")
        logger.info(f'Serial Number: {packet["hw_serial"]}')
        logger.info(f"  == BMS ID-{packet["BMS_Id"]} ===")
        logger.info(f"     == Temp ==")
        logger.info(f"       Temp 1: {packet["temp1"]}c | Temp 2: {packet["temp2"]}c | Temp Mos: {packet["temperature_mos"]}c")
        logger.info(f"       Temp Max: {packet["temp_max"]} | Temp Min: {packet["temp_min"]}")
        logger.info("     == BMS Data ==")
        logger.info("       Voltage: "
            + "%.3fv" % packet["cell_voltage"]
            + " | Current: "
            + str(packet["current"])
            + "A"
        )
        logger.info(f"       Capacity Left: {packet["capacity_remain"]} of {packet["capacity"]} AH")
        logger.info(f"       Balancing State: {packet["balancing_text"]}")
        logger.info(f"       State: {packet["status"]}")
        logger.info(f"       Last Update: {packet["cellLastPoll"]}")
        logger.info("     ===== Warning/Alarms =====")
        logger.info(f"      {packet["warning"]}")
        logger.info(f"      {packet["protection"]}")
        logger.info(f"      {packet["error"]}")
        if alarm_msg:
            logger.info("     ===== Active Alarms =====")
            # Only print alarms with state != 0
            for alarm_name, state in alarm_msg.items():
                if state != 0:
                    level = "WARNING" if state == 1 else "PROTECTION"
                    logger.info(f"       {alarm_name}: {level}")

        logger.info("     == Cell Stats ==")
        cellId = 1
        while cellId <= packet['cell_count']:
            logger.info(f"       Cell {str(cellId)} Voltage: {packet['cell'+str(cellId)]}")
            cellId += 1
        logger.info(f"       Cell Max/Min/Diff: ({packet["cell_max"]}/{packet["cell_min"]}/{round((packet["cell_max"] - packet["cell_min"]), 3)})v")
        return True

    def status_logger(self, packet):
        logger.info("===== HW Info =====")
        logger.info(f'Battery Make/Model: {packet["hw_make"]}')
        logger.info(f'Hardware Version: {packet["hw_version"]}')
        logger.info(f'Serial Number: {packet["hw_serial"]}')
        logger.info("===== Temp =====")
        logger.info(f"Temp 1: {packet["temp1"]}c | Temp 2: {packet["temp2"]}c | Temp Mos: {packet["temperature_mos"]}c")
        logger.info(f"Temp Max: {packet["temp_max"]} | Temp Min: {packet["temp_min"]}")
        logger.info(f'Heater {packet["BMS_Id"]} Status: {packet["heater_state"]}')
        logger.info("===== BMS Data =====")
        logger.info("Voltage: "
            + "%.3fv" % packet["cell_voltage"]
            + " | Current: "
            + str(packet["current"])
            + "A"
        )
        logger.info(f"Capacity Left: {packet["capacity_remain"]} of {packet["capacity"]} AH")
        logger.info(f"SoC: {packet["soc"]}%")
        logger.info(f"SoH: {packet["soh"]}% | Cycle Count: {packet["cycles"]}")
        logger.info("===== Warning/Alarms =====")
        logger.info(f"  {packet["warning"]}")
        logger.info(f"  {packet["protection"]}")
        logger.info(f"  {packet["error"]}")
        logger.info("===== Pack Details =====")
        logger.info(f"  === BMS ID-{packet["BMS_Id"]} ===")
        logger.info(f"  State: {packet["status"]}")
        logger.info(f"  Pack Balancing: {packet["balancing_text"]}")
        logger.info(f"  Last Update: {packet["cellLastPoll"]}")
        logger.info(f"  Pack Voltage: {round((packet["cell_voltage"]),3)}v | Pack Current: {round((packet["current"]),2)}a")
        logger.info("    = Cell Stats =")
        cellId = 1
        while cellId <= packet["cell_count"]:
            logger.info(f"  Cell {str(cellId)} Voltage: {packet['cell'+str(cellId)]}")
            cellId += 1
        logger.info(f"  Cell Max/Min/Diff: ({packet["cell_max"]}/{packet["cell_min"]}/{round((packet["cell_max"] - packet["cell_min"]), 3)})v")
        return True

    def update_alarm_dbus(self, alarm_status):
        self.voltage_high = alarm_status.get("Pack_OV", 0)
        self.voltage_cell_high = alarm_status.get("Cell_OV", 0)
        self.voltage_low = alarm_status.get("Pack_UV", 0)
        self.voltage_cell_low = alarm_status.get("Cell_UV", 0)
        self.charge_fet = alarm_status.get("charge_fet", True)
        self.discharge_fet = alarm_status.get("discharge_fet", True)
        self.current_over = max(
            alarm_status.get("Charge_OC1",0),
            alarm_status.get("Charge_OC2",0),
            alarm_status.get("Discharge_OC1",0),
            alarm_status.get("Discharge_OC2",0),
            alarm_status.get("Load_Short",0)
        )
        self.temp_high_internal = alarm_status.get("Charge_OT",0)
        self.temp_high_discharge = alarm_status.get("Discharge_OT",0)
        self.temp_low_charge = alarm_status.get("Charge_UT",0)


    def lookup_warning(self, packet):
        warning_alarm = ""
        code = packet["warning_hex"]
        if code == "0000":
            warning_alarm += "No Warnings - "+code
        elif code == "0800":
            warning_alarm += "Warning: "+code+" - Charge Under Temp"
            self.temp_high_charge = 1
        else:
            warning_alarm += "Warning: "+code+" - UNKNOWN"
        return warning_alarm

    def lookup_protection(self, packet):
        protection_alarm = ""
        code = packet["protection_hex"]
        if code == "0000":
            protection_alarm += "No Protection Events - "+code
            self.charge_fet = True
            self.discharge_fet = True
        else:
            protection_alarm += "Protection UNKNOWN: "+code
        return protection_alarm

    def lookup_error(self, packet):
        code = packet["error_hex"]
        error_alarm = ""
        if code == "0000":
            error_alarm = f"No Errors - "+code
        elif code == "8000":
            error_alarm = f"Error: "+code+" - Temperature Error"
            logger.error(f"BMS Error: {error_alarm}")
            self.bms_stats(packet)
        else:
            error_alarm = "UNKNOWN: "+code
        return error_alarm

    def lookup_status(self, packet):
        status_code = ""
        if packet["status_hex"] == "0000":
            status_code = "Active/Standby"
        elif packet["status_hex"] == "0100":
            status_code = "Active/Charging"
        elif packet["status_hex"] == "0200":
            status_code = "Active/Discharging"
        elif packet["status_hex"] == "0008":
            status_code = "Inactive/Protect"
            logger.error(f"BMS Status: {status_code}")
            self.balance_status(packet)
        elif packet["status_hex"] == "0800":
            status_code = "Active/Charging Limit"
            logger.error(f"BMS Status: {status_code}")
            self.balance_status(packet)
        else:
            status_code = "UNKNOWN - "+packet["status_hex"]
            logger.error(f"BMS Status: {status_code}")
            self.balance_status(packet)
        return status_code

    def lookup_heater(self, packet):
        if packet["heater_hex"] == "00":
            heater_state = False
        elif packet["heater_hex"] == "80":
            heater_state = True
        return heater_state

    def get_balancing(self):
        balacing_state = self.balancingStat(self.battery_stats[self.Id])
        return balacing_state

    def balancingStat(self, packet):
        balancer_delta = float(packet.get("Balance_Volt_Diff") or 0.040)
        balancer_min_voltage = float(packet.get("Balance_Volt") or 3.40)
        cell_count = packet.get("cell_count")
        cell_min = packet.get("cell_min")
        cell_max = packet.get("cell_max")
        if cell_count is None or cell_min is None or cell_max is None:
            self.balacing_state = 0
            self.balance_fet = False
            return self.balacing_state
        delta_v = round(cell_max - cell_min, 3)
        # ---- Rule 1: all cells below balance voltage ----
        if cell_max < balancer_min_voltage:
            self.balacing_state = 0
            self.balance_fet = False
            return self.balacing_state
        # ---- Rule 3: top balancing finished ----
        if (cell_min >= utils.MAX_CELL_VOLTAGE and delta_v <= balancer_delta):
            if self.balacing_state != 2:
                logger.info(f"BMS {packet["BMS_Id"]} balancing finished")
                self.balance_status(packet)   # your existing callback
            self.balacing_state = 2
            self.balance_fet = False
            return self.balacing_state
        # ---- Rule 2: balancing active ----
        if (cell_min >= balancer_min_voltage and delta_v > balancer_delta):
            self.balacing_state = 1
            self.balance_fet = True
            return self.balacing_state
        # ---- Default: monitoring / idle ----
        self.balacing_state = 0
        self.balance_fet = True
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
        logger.info("Polling BMS for config settings...")
        command = self.eg4CommandGen(self.Id.to_bytes(1, "big") + self.bmsConfigCommandRoot)
        packet = self.read_eg4ll_command(command)
        if not packet:
            logger.error("Failed to read BMS config, will use defaults")
            return False

        # --- Helper functions ---
        def get_int(packet, start, end, signed=False, modifier=1, round_decimals=None):
            if len(packet) < end:
                return None
            value = int.from_bytes(packet[start:end], "big", signed=signed) * modifier
            if round_decimals is not None:
                value = round(value, round_decimals)
            return value
        def get_temp(packet, start, end):
            if len(packet) < end:
                return None
            return int.from_bytes(packet[start:end], "big", signed=True) - 50
        def decode_string(packet, start, end):
            if len(packet) < end:
                return ""
            return packet[start:end].decode("utf-8", errors="ignore").strip('\x00')

        BMSConf = {}
        # --- Voltage thresholds ---
        BMSConf["Balance_Volt"] = get_int(packet, 25, 27, modifier=1/1000, round_decimals=3)
        BMSConf["Balance_Volt_Diff"] = get_int(packet, 27, 29, modifier=1/1000, round_decimals=3)
        BMSConf["Low_Cap_Warn"] = get_int(packet, 29, 31)
        BMSConf["Cell_UV_Warn"] = get_int(packet, 35, 37, modifier=1/1000, round_decimals=3)
        BMSConf["Cell_OV_Warn"] = get_int(packet, 47, 49, modifier=1/1000, round_decimals=3)
        BMSConf["Cell_UV_Protect"] = get_int(packet, 37, 39, modifier=1/1000, round_decimals=3)
        BMSConf["Cell_OV_Protect"] = get_int(packet, 49, 51, modifier=1/1000, round_decimals=3)
        BMSConf["Cell_UV_Release"] = get_int(packet, 39, 41, modifier=1/1000, round_decimals=3)
        BMSConf["Cell_OV_Release"] = get_int(packet, 51, 53, modifier=1/1000, round_decimals=3)
        BMSConf["Pack_UV_Warn"] = get_int(packet, 41, 43, modifier=1/100, round_decimals=2)
        BMSConf["Pack_OV_Warn"] = get_int(packet, 53, 55, modifier=1/100, round_decimals=2)
        BMSConf["Pack_UV_Protect"] = get_int(packet, 43, 45, modifier=1/100, round_decimals=2)
        BMSConf["Pack_OV_Protect"] = get_int(packet, 55, 57, modifier=1/100, round_decimals=2)
        BMSConf["Pack_UV_Release"] = get_int(packet, 45, 47, modifier=1/100, round_decimals=2)
        BMSConf["Pack_OV_Release"] = get_int(packet, 57, 59, modifier=1/100, round_decimals=2)
        # --- Temperature thresholds ---
        temp_fields = [
            ("Charge_UT_Warn", 93, 95),
            ("Charge_UT_Protect", 95, 97),
            ("Charge_UT_Release", 97, 99),
            ("Charge_OT_Warn", 99, 101),
            ("Charge_OT_Protect", 101, 103),
            ("Charge_OT_Release", 103, 105),
            ("Discharge_UT_Warn", 105, 107),
            ("Discharge_UT_Protect", 107, 109),
            ("Discharge_UT_Release", 109, 111),
            ("Discharge_OT_Warn", 111, 113),
            ("Discharge_OT_Protect", 113, 115),
            ("Discharge_OT_Release", 115, 117),
            ("PCB_OT_Warn", 117, 119),
            ("PCB_OT_Protect", 119, 121),
            ("PCB_OT_Release", 121, 123),
        ]
        for suffix, start, end in temp_fields:
            BMSConf[suffix] = get_temp(packet, start, end)
        # --- Current protections and delays ---
        current_fields = [
            ("Charge_OC1_Protect", 73, 75, 1/100),
            ("Charge_OC1_Delay", 83, 85, 1),
            ("Charge_OC2_Protect", 79, 81, 1/100),
            ("Charge_OC2_Delay", 85, 87, 1),
            ("Discharge_OC1_Protect", 75, 77, 1/100),
            ("Discharge_OC1_Delay", 87, 89, 1),
            ("Discharge_OC2_Protect", 81, 83, 1/100),
            ("Discharge_OC2_Delay", 89, 91, 1),
            ("Charge_Release", 71, 73, 1),
            ("Charge_OC_Times", 91, 92, 1),
            ("Discharge_Release", 69, 71, 1),
            ("Discharge_OC_Times", 92, 93, 1),
            ("Load_Short_Current", 77, 79, 1/100),
        ]
        for key, start, end, modifier in current_fields:
            BMSConf[key] = get_int(packet, start, end, modifier=modifier, round_decimals=2)
        BMSConf["hw_make"] = decode_string(packet, 123, 145)
        BMSConf["hw_version"] = decode_string(packet, 146, 153)
        BMSConf["hw_serial"] = decode_string(packet, 153, 167) + str(self.Id)
        BMSConf["hwLastPoll"] = datetime.datetime.now()
        logger.info("BMS Config Pulled Settings:\n%s", pformat(BMSConf))
        return BMSConf

    def eg4CommandGen(self, data: bytes) -> bytes:
        crc = 0xFFFF
        poly = 0xA001
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ poly
                else:
                    crc >>= 1
                crc &= 0xFFFF  # keep 16-bit
        return data + struct.pack("<H", crc)

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
