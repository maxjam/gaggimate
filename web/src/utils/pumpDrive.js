// Fork (meterlink.15): the pumps on the Flipsky dual VESC, as the controller reads them from the CAN bus.
// The display sends them in evt:status as `pd`; ApiService stores that object as status.pumpDrive.
import { msg } from '@lingui/core/macro';
import { i18n } from '../i18n.js';

// 14 magnet poles on the Velofuso motor: mechanical rpm = ERPM / 7.
export const PUMP_POLE_PAIRS = 7;
export const PUMP_VESC_IDS = [18, 19];

export const PUMP_STATE = {
  OFF: 0,
  RUNNING: 1,
  STALL: 2,
  HOT: 3,
  FAULT: 4,
  NO_DATA: 5,
  NOT_USED: 6,
};

const STATE_INFO = {
  [PUMP_STATE.OFF]: { labelMsg: msg`OFF`, tone: 'neutral' },
  [PUMP_STATE.RUNNING]: { labelMsg: msg`RUNNING`, tone: 'good' },
  [PUMP_STATE.STALL]: { labelMsg: msg`STALL`, tone: 'bad' },
  [PUMP_STATE.HOT]: { labelMsg: msg`HOT`, tone: 'bad' },
  [PUMP_STATE.FAULT]: { labelMsg: msg`FAULT`, tone: 'bad' },
  [PUMP_STATE.NO_DATA]: { labelMsg: msg`NO DATA`, tone: 'warn' },
  [PUMP_STATE.NOT_USED]: { labelMsg: msg`NOT USED`, tone: 'neutral' },
};

export function pumpStateInfo(state) {
  const info = STATE_INFO[state] ?? STATE_INFO[PUMP_STATE.NO_DATA];
  return { label: i18n._(info.labelMsg), tone: info.tone };
}

// mc_fault_code, VESC firmware 7.00 (datatypes.h).
const VESC_FAULTS = [
  'NONE',
  'OVER_VOLTAGE',
  'UNDER_VOLTAGE',
  'DRV',
  'ABS_OVER_CURRENT',
  'OVER_TEMP_FET',
  'OVER_TEMP_MOTOR',
  'GATE_DRIVER_OVER_VOLTAGE',
  'GATE_DRIVER_UNDER_VOLTAGE',
  'MCU_UNDER_VOLTAGE',
  'BOOTING_FROM_WATCHDOG_RESET',
  'ENCODER_SPI',
  'ENCODER_SINCOS_BELOW_MIN_AMPLITUDE',
  'ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE',
  'FLASH_CORRUPTION',
  'HIGH_OFFSET_CURRENT_SENSOR_1',
  'HIGH_OFFSET_CURRENT_SENSOR_2',
  'HIGH_OFFSET_CURRENT_SENSOR_3',
  'UNBALANCED_CURRENTS',
  'BRK',
  'RESOLVER_LOT',
  'RESOLVER_DOS',
  'RESOLVER_LOS',
  'FLASH_CORRUPTION_APP_CFG',
  'FLASH_CORRUPTION_MC_CFG',
  'ENCODER_NO_MAGNET',
  'ENCODER_MAGNET_TOO_STRONG',
  'PHASE_FILTER',
  'ENCODER_FAULT',
  'LV_OUTPUT_FAULT',
  'ENCODER_SLIP',
  'OVERSPEED',
  'UNDERSPEED',
  'ABS_OVERSPEED',
];

export const vescFaultName = code => VESC_FAULTS[code] ?? `UNKNOWN`;

// "0 NONE", "4 ABS_OVER_CURRENT"; a dash while the VESC has not answered the fault poll.
export const formatFault = code =>
  code === null || code === undefined ? '—' : `${code} ${vescFaultName(code)}`;

export const erpmToRpm = erpm =>
  erpm === null || erpm === undefined ? null : Math.round(erpm / PUMP_POLE_PAIRS);

export const formatNumber = (value, digits = 0) =>
  value === null || value === undefined || Number.isNaN(value)
    ? '—'
    : value.toLocaleString(undefined, {
        minimumFractionDigits: digits,
        maximumFractionDigits: digits,
      });

// Pump stop codes, as the controller sends them (ERROR_CODE_PUMP_*).
export const PUMP_STOP = { STALL: 50, HOT: 51, FAULT: 52, NO_DATA: 53 };

// One sentence for a pump stop, e.g. "Pump 1 stalled: no rotation at 12 ERPM."
export function pumpStopText(stop) {
  if (!stop || !stop.c) return '';
  const pump = stop.p || 1;
  switch (stop.c) {
    case PUMP_STOP.STALL:
      return i18n._(msg`Pump ${pump} stalled: power on it, but it was not turning.`);
    case PUMP_STOP.HOT: {
      const temperature = stop.d;
      return i18n._(msg`Pump ${pump} motor too hot (${temperature} °C).`);
    }
    case PUMP_STOP.FAULT: {
      const fault = formatFault(stop.d);
      return i18n._(msg`Pump ${pump} VESC fault ${fault}.`);
    }
    case PUMP_STOP.NO_DATA:
      return i18n._(msg`Pump ${pump} drive is not answering on the CAN bus.`);
    default:
      return i18n._(msg`Pump ${pump} was stopped.`);
  }
}

// "just now", "40 s ago", "12 min ago", "3 h ago"
export function formatAgo(seconds) {
  if (seconds === null || seconds === undefined) return '';
  if (seconds < 5) return i18n._(msg`just now`);
  if (seconds < 90) return i18n._(msg`${seconds} s ago`);
  const minutes = Math.round(seconds / 60);
  if (minutes < 90) return i18n._(msg`${minutes} min ago`);
  const hours = Math.round(minutes / 60);
  return i18n._(msg`${hours} h ago`);
}
