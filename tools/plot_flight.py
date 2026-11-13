#!/usr/bin/env python3
"""Plot a logged flight.

  plot_flight.py                 newest flight in <workspace>/data
  plot_flight.py data/flight_20261112_221500
  plot_flight.py --show          also open the figures in windows

Writes PNGs to <flight>/plots/ and prints tracking-error statistics. Uses whatever files the
flight has: PX4 estimates (MAVSDK CSVs), setpoints, Gazebo ground truth and disturbances
(disturbance.csv, SITL only). For the full PX4 log use the .ulg in the folder with pyulog
(ulog2csv) or https://review.px4.io.
"""

import argparse
import os
import sys
from pathlib import Path

try:
    import matplotlib
    import numpy as np
    import pandas as pd
except ImportError:
    sys.exit("needs pandas, numpy and matplotlib:  pip install pandas matplotlib")

WS = Path(__file__).resolve().parent.parent
SETTLE_S = 10.0  # seconds after a new setpoint excluded from the hold statistics


def find_flight(arg):
    if arg:
        path = Path(arg)
        if not path.is_dir():
            sys.exit(f"no such folder: {path}")
        return path
    root = Path(os.environ.get("SUAVE_DATA_DIR", WS / "data"))
    flights = sorted(root.glob("flight_*"))
    if not flights:
        sys.exit(f"no flights in {root}")
    return flights[-1]


def load(folder, name):
    path = folder / f"{name}.csv"
    if not path.exists() or path.stat().st_size == 0:
        return None
    try:
        df = pd.read_csv(path)
    except Exception as exc:  # noqa: BLE001 - report and carry on
        print(f"  skip {path.name}: {exc}")
        return None
    return df if len(df) else None


class Flight:
    def __init__(self, folder):
        self.folder = folder
        names = ["position_velocity_ned", "attitude_euler", "angular_velocity_body", "imu", "actuator_outputs",
                 "actuator_controls", "setpoints", "events", "disturbance", "disturbance_events", "battery", "flight_mode"]
        self.d = {n: load(folder, n) for n in names}
        if all(df is None for df in self.d.values()):
            sys.exit(f"{folder} has no data")

        # Time base: the logger's steady clock "t" (s since suave_main started). Wall-clock
        # t_unix is NOT used: WSL/NTP can step it mid-flight.
        for name, df in self.d.items():
            if df is not None and name not in ("disturbance", "disturbance_events") and "t" in df:
                df["time"] = df["t"]

        # Gazebo files are stamped with sim time, which is also PX4's clock. Map sim time to
        # the logger clock with PX4 message timestamps (attitude / imu carry timestamp_us).
        self.sim_map = None
        for ref in ("attitude_euler", "imu"):
            df = self.d[ref]
            if df is not None and "timestamp_us" in df and df["timestamp_us"].gt(0).sum() > 10:
                good = df[df["timestamp_us"] > 0]
                self.sim_map = np.polyfit(good["timestamp_us"] / 1e6, good["t"], 1)
                break
        for name in ("disturbance", "disturbance_events"):
            df = self.d[name]
            if df is None:
                continue
            if self.sim_map is not None:
                df["time"] = np.polyval(self.sim_map, df["t_sim"])
            else:  # fallback: wall clock, relative to the logger start
                events = self.d["events"]
                if events is not None:
                    df["time"] = df["t_unix"] - events["t_unix"].iloc[0] + events["t"].iloc[0]

    def align_truth(self):
        """Shift Gazebo truth into PX4's local frame (origin where the EKF started).

        Gazebo measures the body origin (~0.2 m above ground on the x500); PX4's local
        origin is where it initialised. The offset is taken on the ground before the first
        setpoint, so any later difference is real estimation error.
        """
        truth, pv, sp = self["disturbance"], self["position_velocity_ned"], self["setpoints"]
        if truth is None or pv is None:
            return None
        t_end = sp["time"].iloc[0] if sp is not None else pv["time"].iloc[0] + 5.0
        pre_truth = truth[truth["time"] < t_end]
        pre_pv = pv[pv["time"] < t_end]
        if len(pre_truth) < 5 or len(pre_pv) < 5:
            return None
        offset = {k: pre_truth[k].mean() - pre_pv[k].mean() for k in ("n_m", "e_m", "d_m")}
        for k, o in offset.items():
            truth[k] = truth[k] - o
        return offset

    def __getitem__(self, name):
        return self.d.get(name)


def add_events(ax_list, flight):
    ev = flight["events"]
    if ev is None:
        return
    dev = flight["disturbance_events"]
    if dev is not None and "time" in dev:
        # When each disturbance actually took effect in the simulation
        for _, row in dev.iterrows():
            for ax in ax_list:
                ax.axvline(row["time"], color="C1", ls=":", lw=0.9, zorder=0)
    marks = ev[ev["kind"].isin(["command", "mark"])]
    for _, row in marks.iterrows():
        for ax in ax_list:
            ax.axvline(row["time"], color="0.6", ls="--", lw=0.7, zorder=0)
        label = str(row["text"])
        label = label if len(label) <= 28 else label[:27] + "…"
        ax_list[0].text(row["time"], 1.0, f" {label}", transform=ax_list[0].get_xaxis_transform(),
                        rotation=90, va="top", ha="left", fontsize=7, color="0.35")


def position_setpoints(flight):
    sp = flight["setpoints"]
    if sp is None:
        return None
    sp = sp[sp["type"] == "position_ned"]
    return sp if len(sp) else None


def figure(rows, title, sharex=True):
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(rows, 1, figsize=(11, 2.4 * rows + 0.8), sharex=sharex, squeeze=False)
    fig.suptitle(title)
    return fig, [a[0] for a in axes]


def save(fig, folder, name, written):
    fig.axes[-1].set_xlabel("time since start [s]")
    fig.tight_layout()
    out = folder / f"{name}.png"
    fig.savefig(out, dpi=110)
    written.append(out)


def plot_all(flight, out, show):
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    written = []
    pv, truth, sp = flight["position_velocity_ned"], flight["disturbance"], position_setpoints(flight)

    # --- position -----------------------------------------------------------------------
    if pv is not None or truth is not None:
        fig, axes = figure(3, "Position NED [m]")
        for ax, (k, kt, ks, lbl) in zip(axes, [("n_m", "n_m", "a", "north"), ("e_m", "e_m", "b", "east"),
                                                ("d_m", "d_m", "c", "down")]):
            if truth is not None:
                ax.plot(truth["time"], truth[kt], color="k", lw=2.2, alpha=0.35, label="Gazebo truth")
            if pv is not None:
                ax.plot(pv["time"], pv[k], color="C0", lw=1, label="PX4 estimate")
            if sp is not None:
                ax.step(sp["time"], sp[ks], where="post", color="C3", lw=1, label="setpoint")
            ax.set_ylabel(lbl)
            ax.grid(alpha=0.3)
        axes[0].legend(loc="upper right", fontsize=8)
        add_events(axes, flight)
        save(fig, out, "position", written)

        # top-down track
        fig, ax = plt.subplots(figsize=(6.5, 6.5))
        if truth is not None:
            ax.plot(truth["e_m"], truth["n_m"], color="k", lw=2.2, alpha=0.35, label="Gazebo truth")
        if pv is not None:
            ax.plot(pv["e_m"], pv["n_m"], color="C0", lw=1, label="PX4 estimate")
        if sp is not None:
            ax.plot(sp["b"], sp["a"], "o", color="C3", ms=5, label="setpoints")
        ax.set_xlabel("east [m]")
        ax.set_ylabel("north [m]")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        ax.set_title("Track (top view)")
        fig.tight_layout()
        fig.savefig(out / "track.png", dpi=110)
        written.append(out / "track.png")

    # --- tracking error vs the active position setpoint ---------------------------------
    stats = None
    all_sp = flight["setpoints"]
    if pv is not None and sp is not None:
        # Only while the active setpoint is a position (not during landing etc.).
        merged = pd.merge_asof(pv.sort_values("time"), all_sp.sort_values("time")[["time", "type", "a", "b", "c"]],
                               on="time", direction="backward")
        merged = merged[merged["type"] == "position_ned"]
        if len(merged):
            err = merged[["n_m", "e_m", "d_m"]].to_numpy() - merged[["a", "b", "c"]].to_numpy()
            fig, axes = figure(2, "Position error vs active setpoint (PX4 estimate)")
            for i, lbl in enumerate(["north", "east", "down"]):
                axes[0].plot(merged["time"], err[:, i], lw=1, label=lbl)
            axes[0].set_ylabel("error [m]")
            axes[0].legend(fontsize=8)
            norm = np.linalg.norm(err, axis=1)
            axes[1].plot(merged["time"], norm, color="C3", lw=1)
            axes[1].set_ylabel("|error| [m]")
            for ax in axes:
                ax.grid(alpha=0.3)
            add_events(axes, flight)
            save(fig, out, "tracking_error", written)
            # Hold performance: skip the first SETTLE_S after every new setpoint (step transients).
            changes = all_sp["time"].to_numpy()
            since = merged["time"].to_numpy()[:, None] - changes[None, :]
            since = np.where(since >= 0, since, np.inf).min(axis=1)
            hold = since > SETTLE_S
            hold_stats = None
            if hold.any():
                eh = err[hold]
                nh = np.linalg.norm(eh, axis=1)
                hold_stats = (np.sqrt(np.mean(eh ** 2, axis=0)), np.sqrt(np.mean(nh ** 2)), nh.max())
            stats = ((np.sqrt(np.mean(err ** 2, axis=0)), np.sqrt(np.mean(norm ** 2)), norm.max()), hold_stats)

    # --- velocity -----------------------------------------------------------------------
    if pv is not None or truth is not None:
        fig, axes = figure(3, "Velocity NED [m/s]")
        for ax, (k, lbl) in zip(axes, [("vn_m_s", "north"), ("ve_m_s", "east"), ("vd_m_s", "down")]):
            if truth is not None:
                ax.plot(truth["time"], truth[k], color="k", lw=2.2, alpha=0.35, label="Gazebo truth")
            if pv is not None:
                ax.plot(pv["time"], pv[k], color="C0", lw=1, label="PX4 estimate")
            ax.set_ylabel(lbl)
            ax.grid(alpha=0.3)
        axes[0].legend(loc="upper right", fontsize=8)
        add_events(axes, flight)
        save(fig, out, "velocity", written)

    # --- attitude and rates -------------------------------------------------------------
    att, rates = flight["attitude_euler"], flight["angular_velocity_body"]
    if att is not None or truth is not None:
        fig, axes = figure(3, "Attitude [deg]")
        for ax, k in zip(axes, ["roll_deg", "pitch_deg", "yaw_deg"]):
            if truth is not None:
                ax.plot(truth["time"], truth[k], color="k", lw=2.2, alpha=0.35, label="Gazebo truth")
            if att is not None:
                ax.plot(att["time"], att[k], color="C0", lw=1, label="PX4 estimate")
            ax.set_ylabel(k.replace("_deg", ""))
            ax.grid(alpha=0.3)
        axes[0].legend(loc="upper right", fontsize=8)
        add_events(axes, flight)
        save(fig, out, "attitude", written)
    if rates is not None or truth is not None:
        fig, axes = figure(3, "Body rates FRD [rad/s]")
        for ax, (k, kt) in zip(axes, [("roll_rad_s", "p_rad_s"), ("pitch_rad_s", "q_rad_s"), ("yaw_rad_s", "r_rad_s")]):
            if truth is not None:
                ax.plot(truth["time"], truth[kt], color="k", lw=2.2, alpha=0.35, label="Gazebo truth")
            if rates is not None:
                ax.plot(rates["time"], rates[k], color="C0", lw=1, label="PX4")
            ax.set_ylabel(kt.split("_")[0])
            ax.grid(alpha=0.3)
        axes[0].legend(loc="upper right", fontsize=8)
        add_events(axes, flight)
        save(fig, out, "rates", written)

    # --- IMU ----------------------------------------------------------------------------
    imu = flight["imu"]
    if imu is not None:
        fig, axes = figure(2, "IMU (HIGHRES_IMU, body FRD)")
        for k, lbl in [("ax_m_s2", "forward"), ("ay_m_s2", "right"), ("az_m_s2", "down")]:
            axes[0].plot(imu["time"], imu[k], lw=0.8, label=lbl)
        for k, lbl in [("gx_rad_s", "roll"), ("gy_rad_s", "pitch"), ("gz_rad_s", "yaw")]:
            axes[1].plot(imu["time"], imu[k], lw=0.8, label=lbl)
        axes[0].set_ylabel("accel [m/s²]")
        axes[1].set_ylabel("gyro [rad/s]")
        for ax in axes:
            ax.legend(fontsize=8, loc="upper right")
            ax.grid(alpha=0.3)
        add_events(axes, flight)
        save(fig, out, "imu", written)

    # --- motors -------------------------------------------------------------------------
    act = flight["actuator_outputs"]
    if act is not None:
        cols = [c for c in act.columns if c.startswith("out") and act[c].notna().any() and (act[c] != 0).any()]
        if cols:
            fig, axes = figure(1, "Actuator outputs")
            for c in cols[:8]:
                axes[0].plot(act["time"], act[c], lw=0.8, label=c)
            axes[0].legend(fontsize=8, ncol=4)
            axes[0].grid(alpha=0.3)
            add_events(axes, flight)
            save(fig, out, "motors", written)

    # --- disturbances -------------------------------------------------------------------
    if truth is not None:
        fig, axes = figure(4, "Disturbances (Gazebo, NED)")
        for k, lbl in [("wind_n", "north"), ("wind_e", "east"), ("wind_d", "down")]:
            axes[0].plot(truth["time"], truth[k], lw=1, label=lbl)
        axes[0].set_ylabel("wind [m/s]")
        for k, lbl in [("airspeed_n", "north"), ("airspeed_e", "east"), ("airspeed_d", "down")]:
            axes[1].plot(truth["time"], truth[k], lw=1, label=lbl)
        axes[1].set_ylabel("airspeed [m/s]")
        for k, lbl in [("f_wind_n", "wind N"), ("f_wind_e", "wind E"), ("f_wind_d", "wind D"),
                       ("f_push_n", "push N"), ("f_push_e", "push E"), ("f_push_d", "push D")]:
            if (truth[k] != 0).any():
                axes[2].plot(truth["time"], truth[k], lw=1, label=lbl)
        axes[2].set_ylabel("force [N]")
        for k, lbl in [("tau_x_frd", "roll"), ("tau_y_frd", "pitch"), ("tau_z_frd", "yaw")]:
            axes[3].plot(truth["time"], truth[k], lw=1, label=lbl)
        axes[3].set_ylabel("torque [N m]")
        for ax in axes:
            handles, _ = ax.get_legend_handles_labels()
            if handles:
                ax.legend(fontsize=8, loc="upper right", ncol=3)
            ax.grid(alpha=0.3)
        add_events(axes, flight)
        save(fig, out, "disturbance", written)

    if show:
        plt.show()
    plt.close("all")
    return written, stats


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("flight", nargs="?", help="flight folder (default: newest)")
    parser.add_argument("--show", action="store_true", help="open the figures")
    parser.add_argument("--no-align", action="store_true",
                        help="plot Gazebo truth in its own frame instead of PX4's local frame")
    args = parser.parse_args()

    folder = find_flight(args.flight)
    print(f"Flight: {folder}")
    flight = Flight(folder)
    if not args.no_align:
        offset = flight.align_truth()
        if offset:
            print("Gazebo truth aligned to PX4 local frame (offset N {n_m:+.3f} E {e_m:+.3f} D {d_m:+.3f} m)".format(**offset))
    out = folder / "plots"
    out.mkdir(exist_ok=True)
    written, stats = plot_all(flight, out, args.show)
    for path in written:
        print(f"  {path}")
    if stats:
        for label, st in (("all, incl. step transients", stats[0]), (f"holding (>{SETTLE_S:.0f} s after each setpoint)", stats[1])):
            if st is None:
                continue
            rms, rms_norm, max_norm = st
            print(f"Position error vs setpoint, {label}: RMS N {rms[0]:.3f}  E {rms[1]:.3f}  D {rms[2]:.3f} m,"
                  f"  |e| RMS {rms_norm:.3f} m, max {max_norm:.3f} m")
    if flight["disturbance"] is None:
        print("(no disturbance.csv: Gazebo truth/disturbance plots need the sim started with tools/sim.sh)")


if __name__ == "__main__":
    main()
