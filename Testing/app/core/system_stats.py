"""
app/core/system_stats.py
------------------------
Author: SUDHARSAN
System metrics reader (CPU usage, RAM utilization, Raspberry Pi temperature).
"""

import os
import sys
import psutil


class SystemStatsMonitor:
    """Monitors system resource utilization and thermal metrics."""

    def __init__(self):
        # Warmup psutil CPU calculation
        psutil.cpu_percent(interval=None)

    def get_cpu_usage(self) -> float:
        """Return overall CPU usage percentage."""
        try:
            return float(psutil.cpu_percent(interval=None))
        except Exception:
            return 0.0

    def get_ram_usage(self) -> float:
        """Return system RAM usage percentage."""
        try:
            return float(psutil.virtual_memory().percent)
        except Exception:
            return 0.0

    def get_cpu_temperature(self) -> float:
        """
        Return CPU core temperature in degrees Celsius.
        Checks Raspberry Pi sysfs path first, then psutil sensors.
        """
        # Raspberry Pi Linux thermal zone path
        pi_thermal_path = "/sys/class/thermal/thermal_zone0/temp"
        if os.path.exists(pi_thermal_path):
            try:
                with open(pi_thermal_path, "r") as f:
                    temp_milli = int(f.read().strip())
                    return temp_milli / 1000.0
            except Exception:
                pass

        # Fallback to psutil sensors
        try:
            temps = psutil.sensors_temperatures()
            if temps:
                for key, entries in temps.items():
                    for entry in entries:
                        if entry.current > 0:
                            return float(entry.current)
        except Exception:
            pass

        return 0.0  # Unavailable (e.g. standard desktop Windows VM without thermal sensor)
