#!/usr/bin/env python3


co2eq_base, tvoc_base = sgp30.baseline_co2eq, sgp30.baseline_tvoc
file = open('/lib/calibration.py', 'w')
file.write(f"calibration=\{'co2eq_base': {co2eq_base}, 'tvoc_base': {tvoc_base}\}")
