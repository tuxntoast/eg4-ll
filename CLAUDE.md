# EG4-LL BMS Driver — Claude Code Context

## What This Project Is
Venus OS `dbus-serialbattery` driver for two EG4-LL 12V 400Ah LiFePO4 batteries on a Cerbo GX.
- Local driver: `egll.py` (this repo)
- Remote target: `root@10.0.1.244:/data/apps/dbus-serialbattery/bms/eg4_ll.py`
- SSH key: `~/.ssh/id_ed25519` (no passphrase)
- Fork for upstream PR: `/Users/patfitz/Documents/GitHub/venus-os_dbus-serialbattery`
  - Branch: `fix/eg4_ll-improvements`
  - Target upstream: `mr-manuel/venus-os_dbus-serialbattery` master

## Hardware Layout
Two BMS units daisy-chained on RS-485 (9600 baud, CH341 USB adapter):

| BMS Address | DeviceInstance | CustomName  | Serial           |
|-------------|----------------|-------------|------------------|
| `0x01`      | 1              | `EG4-LL:01` | `4S1240019050001`  |
| `0x10`      | 2              | `EG4-LL:16` | `4S12400190500016` |

Lynx Shunt 1000A VE.Can: `com.victronenergy.battery.socketcan_can0_vi0_uc171043`

## Deploy Workflow (run after every code change)
```bash
scp egll.py root@10.0.1.244:/data/apps/dbus-serialbattery/bms/eg4_ll.py \
  && ssh root@10.0.1.244 '/data/apps/dbus-serialbattery/restart.sh'
# Wait ~60s for CH341 to settle, then verify:
ssh root@10.0.1.244 'tail -n 40 /data/log/dbus-serialbattery.ttyUSB0/current | tai64nlocal'
```
**Success looks like:** both BMS print `Battery EG4-LL connected to dbus` and `CHARGE FET: True | DISCHARGE FET: True`

**Keep fork in sync** — after every egll.py change, copy to fork and commit:
```bash
cp egll.py /Users/patfitz/Documents/GitHub/venus-os_dbus-serialbattery/dbus-serialbattery/bms/eg4_ll.py
# commit in both repos, update CHANGELOG.md in the fork
```

## Session Resume Checklist
When starting a new session, run these to orient:
```bash
# 1. Local state
git -C /Users/patfitz/Documents/GitHub/eg4-ll log --oneline -5
git -C /Users/patfitz/Documents/GitHub/eg4-ll status

# 2. Fork state
git -C /Users/patfitz/Documents/GitHub/venus-os_dbus-serialbattery log --oneline -5

# 3. Live BMS status on Cerbo
ssh root@10.0.1.244 'tail -n 30 /data/log/dbus-serialbattery.ttyUSB0/current | tai64nlocal'

# 4. PR status
gh pr view 408 --repo mr-manuel/venus-os_dbus-serialbattery
```

## Upstream PR
- **PR #408 OPEN, awaiting review**: https://github.com/mr-manuel/venus-os_dbus-serialbattery/pull/408
- Branch: `tuxntoast:fix/eg4_ll-improvements` → `mr-manuel:master`
- All CI checks passing (CodeQL, Black, flake8, tests)
- Always run `black dbus-serialbattery/bms/eg4_ll.py` in the fork before pushing — CI will fail otherwise

## Key config.ini Settings (deployed to Cerbo)
| Setting | Value | Notes |
|---------|-------|-------|
| `MAX_BATTERY_CHARGE_CURRENT` | 100.0 | |
| `MAX_BATTERY_DISCHARGE_CURRENT` | 200.0 | |
| `MAX_CELL_VOLTAGE` | 3.550 | |
| `FLOAT_CELL_VOLTAGE` | 3.375 | |
| `MIN_CELL_VOLTAGE` | 2.500 | |
| `SOC_RESET_CELL_VOLTAGE` | 3.600 | Safety margin below BMS OV warning |
| `SOC_RESET_AFTER_DAYS` | *(empty)* | Disabled — avoids framework validation warning |
| `CVL_CONTROLLER_MODE` | 2 | I-Controller |
| `CVCM_ENABLE` | True | |
| `POLL_INTERVAL` | 3 | |
| `SOC_CALCULATION` | True | |
| `VOLTAGE_DROP` | 0.00 | I-controller compensates BMS/shunt offset automatically |
| `TEMPERATURES_WHILE_CHARGING` | `0,2,5,10,40,45,50` | Cuts to 0A at 50°C (aligns with BMS OT protection) |

## Architecture Notes
- **CH341 settling**: adapter needs ~40s after port open; driver holds port open between rounds. All serial errors during settling are logged at DEBUG via `_connecting` flag — startup log goes straight to "connected".
- **`Protection` class attributes**: `high_voltage`, `high_cell_voltage`, `low_voltage`, `low_cell_voltage`, `low_soc`, `high_charge_current`, `high_discharge_current`, `high_charge_temperature`, `low_charge_temperature`, `high_temperature`, `low_temperature`, `high_internal_temperature`
- **DeviceInstance** stored at: `/Settings/Devices/serialbattery_<serial>/ClassAndVrmInstance`
- **CustomName** stored at: `/Settings/Devices/serialbattery_<serial>/CustomName`
- **BusyBox on Cerbo**: avoid `--include` grep flag; use `head -n N` not `head -N`

## Known Behaviors (not bugs — don't try to fix these)
- `SocCalc` dbus error on startup: known Victron framework bug, not our code
- Temps 3 & 4 only appear if non-zero (BMS returns 0 for unpopulated sensors)
- Driver takes 1–3 polling rounds (60s each) to connect — normal CH341 settling
- Lynx Shunt reads ~0.04V lower than BMS — expected; I-controller compensates via CVL
- BMS reports 0A for currents below ~1A per battery — hardware dead zone, not fixable in software
