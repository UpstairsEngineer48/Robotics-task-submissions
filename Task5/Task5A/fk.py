#!/usr/bin/env python3
import math


def forward_kinematics(wl, wr, wb):
    r = 0.05
    L = 0.680
    vx = r*(wr+wb-2*wl)/3.0
    vy = r*(wr-wb)/math.sqrt(3)
    w  = r*(wl+wr+wb)/(3.0*L)

    return vx,vy,w


def main():

    wl = float(input("Left wheel (rad/s): "))
    wr = float(input("Right wheel (rad/s): "))
    wb = float(input("Rear wheel (rad/s): "))

    vx, vy, omega = forward_kinematics(
        wl,
        wr,
        wb
    )

    print(f"\nvx     : {vx:.3f} m/s")
    print(f"vy     : {vy:.3f} m/s")
    print(f"omega  : {omega:.3f} rad/s")


if __name__ == "__main__":
    main()
