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
        self.bms_stats_cb = bms_stats_cb or (lambda msg: None)
        self.alarm_states = {}        # Tracks previous state for edge-triggered logging
        self.charge_fet = True
        self.discharge_fet = True
        self._charge_oc_time = {}     # timestamps for Charge OC1/OC2
        self._discharge_oc_time = {}  # timestamps for Discharge OC1/OC2

    def _get(self, key):
        """Return value from dictionary, fallback to defaults."""
        return self.data.get(key, self.DEFAULTS.get(key, 0))

    def evaluate(self, new_data=None):
        """
        Evaluate all alarms, update alarm states, FETs, and edge-triggered logging.
        Optional `new_data` can be passed to refresh telemetry.
        """
        if new_data:
            self.data = new_data
        if not self.data:
            return

        now = time.time()
        alarms = {}

        # --- Cell Voltage ---
        cell_max = self.data.get("cell_max", 0)
        cell_min = self.data.get("cell_min", 0)
        alarms["Cell_OV"] = 2 if cell_max >= self._get("Cell_OV_Protect") else 1 if cell_max >= self._get("Cell_OV_Warn") else 0
        alarms["Cell_UV"] = 2 if cell_min <= self._get("Cell_UV_Protect") else 1 if cell_min <= self._get("Cell_UV_Warn") else 0

        # --- Pack Voltage ---
        pack_v = self.data.get("cell_voltage", 0)
        alarms["Pack_OV"] = 2 if pack_v >= self._get("Pack_OV_Protect") else 1 if pack_v >= self._get("Pack_OV_Warn") else 0
        alarms["Pack_UV"] = 2 if pack_v <= self._get("Pack_UV_Protect") else 1 if pack_v <= self._get("Pack_UV_Warn") else 0

        # --- Temperature ---
        temp_max = self.data.get("temp_max", 0)
        temp_min = self.data.get("temp_min", 0)
        alarms["Charge_UT"] = 2 if temp_min <= self._get("Charge_UT_Protect") else 1 if temp_min <= self._get("Charge_UT_Warn") else 0
        alarms["Charge_OT"] = 2 if temp_max >= self._get("Charge_OT_Protect") else 1 if temp_max >= self._get("Charge_OT_Warn") else 0
        alarms["Discharge_UT"] = 2 if temp_min <= self._get("Discharge_UT_Protect") else 1 if temp_min <= self._get("Discharge_UT_Warn") else 0
        alarms["Discharge_OT"] = 2 if temp_max >= self._get("Discharge_OT_Protect") else 1 if temp_max >= self._get("Discharge_OT_Warn") else 0
        alarms["PCB_OT"] = 2 if temp_max >= self._get("PCB_OT_Protect") else 1 if temp_max >= self._get("PCB_OT_Warn") else 0

        # --- Low Capacity ---
        soc = self.data.get("soc", 100)
        alarms["Low_Cap"] = 2 if soc <= self._get("Low_Cap_Warn") else 0

        # --- Load Short ---
        current = self.data.get("current", 0)
        alarms["Load_Short"] = 2 if abs(current) >= self._get("Load_Short_Current") else 0

        return alarms

        # --- Charge/Discharge Overcurrent Timers ---
        def check_overcurrent(value, oc1_prot, oc1_delay, oc2_prot, oc2_delay, last_oc_time):
            state = 0
            if value >= oc2_prot:
                if "OC2_start" not in last_oc_time:
                    last_oc_time["OC2_start"] = now
                if now - last_oc_time["OC2_start"] >= oc2_delay:
                    state = 2
            else:
                last_oc_time.pop("OC2_start", None)

            if value >= oc1_prot:
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
        for alarm, state in alarms.items():
            prev_state = self.alarm_states.get(alarm, 0)
            if state != prev_state:
                #if state == 1:
                #    self.bms_stats_cb(f"{alarm}: WARNING")
                #elif state == 2:
                #    self.bms_stats_cb(f"{alarm}: PROTECTION")
                #elif state == 0 and prev_state in (1, 2):
                #    self.bms_stats_cb(f"{alarm}: CLEARED")
                self.alarm_states[alarm] = state

        # --- FET control flags ---
        self.charge_fet = all(alarms.get(k,0) < 2 for k in ["Cell_OV","Pack_OV","Charge_OT","Charge_UT","Load_Short","Over_Charge_Current"])
        self.discharge_fet = all(alarms.get(k,0) < 2 for k in ["Cell_UV","Pack_UV","Discharge_OT","Discharge_UT","Load_Short","Over_Discharge_Current"])
