#!/usr/bin/env python3

import math


def inverse_kinematics(vx, vy, omega):

    r = 0.05
    L = 0.680

    wl = (-vx+L*omega)/r
    wr = (0.5*vx+(math.sqrt(3)/2.0)*vy+L*omega)/ r
    wb = (0.5*vx-(math.sqrt(3)/2.0)*vy+L*omega)/ r

    return wl, wr, wb


def main():

    vx = float(input("vx (m/s): "))
    vy = float(input("vy (m/s): "))
    omega = float(input("omega (rad/s): "))

    wl, wr, wb = inverse_kinematics(vx, vy, omega)

    print(f"\nLeft wheel  : {wl:.3f} rad/s")
    print(f"Right wheel : {wr:.3f} rad/s")
    print(f"Rear wheel  : {wb:.3f} rad/s")


if __name__ == "__main__":
    main()
