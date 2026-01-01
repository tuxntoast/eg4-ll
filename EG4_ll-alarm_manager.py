import time

class EG4AlarmManager:
    DEFAULTS = {
        "Cell_OV_Warn": 3.6,
        "Cell_OV_Protect": 3.9,
        "Cell_OV_Release": 3.45,
        "Cell_UV_Warn": 2.5,
        "Cell_UV_Protect": 2.0,
        "Cell_UV_Release": 3.1,
        "Charge_UT_Warn": 2,
        "Charge_UT_Protect": 0,
        "Charge_UT_Release": 5,
        "Charge_OT_Warn": 70,
        "Charge_OT_Protect": 75,
        "Charge_OT_Release": 65,
        "Discharge_UT_Warn": -15,
        "Discharge_UT_Protect": -20,
        "Discharge_UT_Release": -10,
        "Discharge_OT_Warn": 70,
        "Discharge_OT_Protect": 75,
        "Discharge_OT_Release": 65,
        "PCB_OT_Warn": 95,
        "PCB_OT_Protect": 100,
        "PCB_OT_Release": 80,
        "Charge_OC1_Protect": 220,
        "Charge_OC1_Delay": 10,
        "Charge_OC2_Protect": 250,
        "Charge_OC2_Delay": 2,
        "Charge_OC_Release": 60,
        "Discharge_OC1_Protect": 220,
        "Discharge_OC1_Delay": 10,
        "Discharge_OC2_Protect": 250,
        "Discharge_OC2_Delay": 2,
        "Discharge_OC_Release": 60,
        "Load_Short_Current": 350,
        "Low_Cap_Warn": 5
    }

    def __init__(self, bms_data, bms_stats_cb=None):
        """
        Initialize the alarm manager.

        :param bms_data: Dictionary containing BMS telemetry & limit values.
        :param bms_stats_cb: Optional callback function for warnings/protections.
        """
        self.data = bms_data
        self.bms_stats_cb = bms_stats_cb
        self.alarm_states = {}        # Tracks previous state for edge-triggered logging
        self.charge_fet = True
        self.discharge_fet = True
        self._initialized = False
        self._charge_oc_time = {}     # timestamps for Charge OC1/OC2
        self._discharge_oc_time = {}  # timestamps for Discharge OC1/OC2

    def _get(self, key):
        if key in self.data:
            return self.data[key]
        if key in self.DEFAULTS:
            return self.DEFAULTS[key]
        return None

    def _check_high(self, value, warn, protect):
        if protect is not None and value >= protect:
            return 2
        if warn is not None and value >= warn:
            return 1
        return 0

    def _check_low(self, value, warn, protect):
        if protect is not None and value <= protect:
            return 2
        if warn is not None and value <= warn:
            return 1
        return 0

    def evaluate(self, new_data=None):
        """
        Evaluate all alarms, update alarm states, FETs, and edge-triggered logging.
        Optional `new_data` can be passed to refresh telemetry.
        """
        if new_data:
            self.data = new_data
        if not self.data:
            return

        # --- Do not evaluate until live telemetry exists ---
        required_keys = ("cell_max", "cell_min", "temp_max", "temp_min", "current", "cell_voltage", "soc")
        if not all(k in self.data for k in required_keys):
            if not hasattr(self, "_telemetry_warned"):
                self.bms_stats_cb({
                    "info": "Waiting for live telemetry before evaluating alarms"
                })
                self._telemetry_warned = True
            return {}

        now = time.time()
        alarms = {}

        if not hasattr(self, "_limits_checked"):
            for k in ["Pack_OV_Protect", "Pack_UV_Protect"]:
                if self._get(k) is None:
                    print(f"[EG4AlarmManager] {k} not provided by BMS — alarm disabled")
            self._limits_checked = True

        # --- Cell Voltage ---
        cell_max = self.data.get("cell_max", 0)
        cell_min = self.data.get("cell_min", 0)
        alarms["Cell_OV"] = self._check_high(cell_max, self._get("Cell_OV_Warn"), self._get("Cell_OV_Protect"))
        alarms["Cell_UV"] = self._check_low(cell_min, self._get("Cell_UV_Warn"), self._get("Cell_UV_Protect"))

        # --- Pack Voltage ---
        pack_v = self.data.get("cell_voltage", 0)
        alarms["Pack_OV"] = self._check_high(pack_v, self._get("Pack_OV_Warn"), self._get("Pack_OV_Protect"))
        alarms["Pack_UV"] = self._check_low(pack_v, self._get("Pack_UV_Warn"), self._get("Pack_UV_Protect"))

        # --- Temperature ---
        temp_max = self.data.get("temp_max", 0)
        temp_min = self.data.get("temp_min", 0)
        alarms["Charge_UT"] = self._check_low(temp_min, self._get("Charge_UT_Warn"), self._get("Charge_UT_Protect"))
        alarms["Charge_OT"] = self._check_high(temp_max, self._get("Charge_OT_Warn"),self._get("Charge_OT_Protect"))
        alarms["Discharge_UT"] = self._check_low(temp_min, self._get("Discharge_UT_Warn"), self._get("Discharge_UT_Protect"))
        alarms["Discharge_OT"] = self._check_high(temp_max, self._get("Discharge_OT_Warn"), self._get("Discharge_OT_Protect"))
        alarms["PCB_OT"] = self._check_high(temp_max, self._get("PCB_OT_Warn"), self._get("PCB_OT_Protect"))

        # --- Low Capacity ---
        soc = self.data.get("soc", 100)
        low_cap = self._get("Low_Cap_Warn")
        alarms["Low_Cap"] = 2 if low_cap is not None and soc <= low_cap else 0

        # --- Load Short ---
        current = self.data.get("current", 0)
        ls = self._get("Load_Short_Current")
        alarms["Load_Short"] = 2 if ls is not None and abs(current) >= ls else 0

        # --- Charge/Discharge Overcurrent Timers ---
        def check_overcurrent(value, oc1_prot, oc1_delay, oc2_prot, oc2_delay, last_oc_time):
            state = 0

            if oc2_prot is not None and value >= oc2_prot:
                if "OC2_start" not in last_oc_time:
                    last_oc_time["OC2_start"] = now
                if now - last_oc_time["OC2_start"] >= oc2_delay:
                    state = 2
            else:
                last_oc_time.pop("OC2_start", None)

            if oc1_prot is not None and value >= oc1_prot:
                if "OC1_start" not in last_oc_time:
                    last_oc_time["OC1_start"] = now
                if now - last_oc_time["OC1_start"] >= oc1_delay:
                    state = max(state, 2)
            else:
                last_oc_time.pop("OC1_start", None)

            return state


        alarms["Over_Charge_Current"] = check_overcurrent(
            max(current, 0),
            self._get("Charge_OC1_Protect"),
            self._get("Charge_OC1_Delay"),
            self._get("Charge_OC2_Protect"),
            self._get("Charge_OC2_Delay"),
            self._charge_oc_time
        )

        alarms["Over_Discharge_Current"] = check_overcurrent(
            abs(min(current, 0)),
            self._get("Discharge_OC1_Protect"),
            self._get("Discharge_OC1_Delay"),
            self._get("Discharge_OC2_Protect"),
            self._get("Discharge_OC2_Delay"),
            self._discharge_oc_time
        )

        # --- Edge-triggered logging ---
        changed = False
        for alarm, state in alarms.items():
            prev = self.alarm_states.get(alarm, state)
            if state != prev:
                changed = True
            self.alarm_states[alarm] = state

        # Suppress logging on first valid evaluation
        if not self._initialized:
            self._initialized = True
            return alarms

        if changed and self.bms_stats_cb:
            active_alarms = {k: v for k, v in alarms.items() if v != 0}
            if active_alarms:
                self.bms_stats_cb(self.data, active_alarms)

        # --- FET control flags ---
        # Charge FET logic
        self.charge_fet = all(
            alarms.get(k, 0) < 2 for k in ["Cell_OV", "Pack_OV", "Charge_OT", "Charge_UT", "Load_Short", "Over_Charge_Current"]
        )
        # Discharge FET logic
        self.discharge_fet = all(
            alarms.get(k, 0) < 2 for k in ["Cell_UV", "Pack_UV", "Discharge_OT", "Discharge_UT", "Load_Short", "Over_Discharge_Current"]
        )

        # --- Map alarms to warning/protection variables ---
        self.voltage_high = max([alarms.get("Pack_OV", 0), alarms.get("Cell_OV", 0)])
        self.voltage_cell_high = alarms.get("Cell_OV", 0)
        self.voltage_low = max([alarms.get("Pack_UV", 0), alarms.get("Cell_UV", 0)])
        self.voltage_cell_low = alarms.get("Cell_UV", 0)
        self.current_over = max([alarms.get("Over_Charge_Current", 0), alarms.get("Over_Discharge_Current", 0)])
        self.temp_high_internal = alarms.get("PCB_OT", 0)
        self.temp_high_discharge = alarms.get("Discharge_OT", 0)
        self.temp_low_charge = alarms.get("Charge_UT", 0)

        return alarms
