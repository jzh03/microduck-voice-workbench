"""Health gate for this MicroDuck's confirmed direct 2S battery supply.

This is the MicroDuck operating profile, not the XL330 manufacturer's rating.
The official runtime pins Shutdown=52 and leaves Max Voltage Limit at 7.0 V:
https://github.com/pollen-robotics/microduck/blob/53230463927095ea890f231d29089bb0a538c06f/duck-control/src/model.rs
No EEPROM protection registers are changed by this module.
"""
import math

SUPPORT_IDS = (10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33)
MIN_VOLTAGE = 6.6
MAX_VOLTAGE = 8.45


def support_health(health):
    issues = []
    expected_voltage_alerts = []
    volts = []
    temperatures = []
    for sid in SUPPORT_IDS:
        h = health.get(str(sid), health.get(sid, {}))
        try:
            voltage = float(h['voltage'])
            temperature = float(h['temperature_c'])
            error = h['hardware_error']
            shutdown = h['shutdown']
            if not math.isfinite(voltage) or not math.isfinite(temperature):
                raise ValueError('non-finite telemetry')
            if type(error) is not int or not 0 <= error <= 255 or type(shutdown) is not int:
                raise ValueError('invalid status')
        except (KeyError, ValueError, TypeError):
            issues.append(f'ID {sid} 健康读数不完整')
            continue
        volts.append(voltage)
        temperatures.append(temperature)
        if shutdown != 52:
            issues.append(f'ID {sid} 保护配置与 MicroDuck 直供方案不符')
        if not MIN_VOLTAGE <= voltage <= MAX_VOLTAGE:
            issues.append(f'ID {sid} 电压 {voltage:.1f} V 超出 6.6–8.45 V')
        if temperature >= 50:
            issues.append(f'ID {sid} 温度 {temperature:g} °C 达到停止门槛')
        if error & ~1:
            issues.append(f'ID {sid} 设备故障 0x{error:02x}')
        elif error & 1 and shutdown == 52 and MIN_VOLTAGE <= voltage <= MAX_VOLTAGE:
            expected_voltage_alerts.append(sid)
    return {
        'ok': not issues,
        'issues': issues,
        'expected_voltage_alert_ids': expected_voltage_alerts,
        'voltage_min': min(volts) if volts else None,
        'voltage_max': max(volts) if volts else None,
        'temperature_max': max(temperatures) if temperatures else None,
        'supply': 'MicroDuck 两节电池直供',
    }
