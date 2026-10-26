#!/usr/bin/env python3
"""Графики результатов моделирования (аналог OS4dynamics_plot_01.m).

Использование:
    python3 tools/plot_os4.py os4sim.csv [os4sim.png]
Нужен пакет matplotlib.
"""
import csv
import math
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load(file_name):
    with open(file_name, newline="") as f:
        rows = list(csv.DictReader(f))
    return {key: [float(r[key]) for r in rows] for key in rows[0]}


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    d = load(sys.argv[1])
    out = sys.argv[2] if len(sys.argv) > 2 else "os4sim.png"
    t = d["time"]
    deg = 180.0 / math.pi

    fig, ax = plt.subplots(3, 2, figsize=(12, 10), sharex=True)
    ax[0][0].plot(t, d["Xe_x"], label="x")
    ax[0][0].plot(t, d["Xe_y"], label="y")
    ax[0][0].plot(t, [-z for z in d["Xe_z"]], label="h = -z")
    ax[0][0].set_title("Положение, м")

    ax[0][1].plot(t, d["Ve_x"], label="Vx")
    ax[0][1].plot(t, d["Ve_y"], label="Vy")
    ax[0][1].plot(t, d["Ve_z"], label="Vz")
    ax[0][1].set_title("Скорость в земной СК, м/с")

    ax[1][0].plot(t, [v * deg for v in d["roll"]], label="крен")
    ax[1][0].plot(t, [v * deg for v in d["pitch"]], label="тангаж")
    ax[1][0].plot(t, [v * deg for v in d["yaw"]], label="курс")
    ax[1][0].set_title("Углы Эйлера 3-2-1, град")

    ax[1][1].plot(t, d["p"], label="p")
    ax[1][1].plot(t, d["q"], label="q")
    ax[1][1].plot(t, d["r"], label="r")
    ax[1][1].set_title("Угловая скорость, рад/с")

    for i in range(1, 5):
        ax[2][0].plot(t, d["w%d" % i], label="w%d" % i)
        ax[2][1].plot(t, d["T%d" % i], label="T%d" % i)
    ax[2][0].set_title("Скорость винтов, рад/с")
    ax[2][1].set_title("Тяга винтов, Н")

    for row in ax:
        for a in row:
            a.grid(True)
            a.legend(loc="best", fontsize=8)
    ax[2][0].set_xlabel("t, с")
    ax[2][1].set_xlabel("t, с")
    fig.tight_layout()
    fig.savefig(out, dpi=100)
    print("saved", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
