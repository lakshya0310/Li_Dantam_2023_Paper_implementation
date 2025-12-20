#!/usr/bin/env python3

import sys
import time
import numpy as np

# -------------------------------
# SELECT WHICH SIMULATION TO USE
# -------------------------------
# Uncomment ONE of these:

from simulation1 import sdf
# from simulation2 import sdf

# -------------------------------
# IMPORTANT NOTES
# -------------------------------
# 1. The simulation file MUST:
#    - initialize PyBullet
#    - load robot and obstacles
#    - define sdf(v): returns signed distance
#
# 2. This script acts as an SDF oracle:
#    - reads joint values from stdin
#    - writes distance to stdout
#
# -------------------------------

def main():
    # Make stdout unbuffered (important for IPC)
    sys.stdout.reconfigure(line_buffering=True)

    while True:
        line = sys.stdin.readline()
        if not line:
            break

        try:
            # Parse joint values
            q = list(map(float, line.strip().split()))

            # Evaluate SDF
            dist = sdf(q)

            # Write result
            print(dist, flush=True)

        except Exception as e:
            # Fail-safe: return large positive distance
            print(1.0, flush=True)

        # Allow PyBullet to advance cleanly
        time.sleep(0.001)

if __name__ == "__main__":
    main()
