#!/usr/bin/env python3
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def get_col(df, *names, required=True):
    for n in names:
        if n in df.columns:
            return df[n]
    if required:
        raise KeyError(f"missing one of columns: {names}")
    return None


def pick_x(df):
    if "ms" in df.columns:
        return "ms"
    if "epoch" in df.columns:
        return "epoch"
    df["ms"] = np.arange(len(df))
    return "ms"


def normalize_columns(df):
    aliases = {
        "E_mag": ["E_mag", "E"],
        "P_mag": ["P_mag", "P"],
        "L_mag": ["L_mag", "L"],
        "pll_disc": ["pll_disc", "pll", "carrier_phase_error"],
        "dll_disc": ["dll_disc", "dll", "code_error"],
        "doppler": ["doppler", "doppler_hz"],
        "carrier_nco_hz": ["carrier_nco_hz", "carrier_freq", "carrier_nco"],
        "code_nco_hz": ["code_nco_hz", "code_freq", "code_nco"],
        "is_locked": ["is_locked", "locked", "lock"],
    }

    for canonical, names in aliases.items():
        if canonical not in df.columns:
            for n in names:
                if n in df.columns:
                    df[canonical] = df[n]
                    break

    for c in ["Ei", "Eq", "Pi", "Pq", "Li", "Lq", "E_mag", "P_mag", "L_mag"]:
        if c in df.columns:
            df[f"{c}_k"] = df[c] / 1000.0

    if {"P_mag", "E_mag", "L_mag"}.issubset(df.columns):
        df["P_minus_E"] = df["P_mag"] - df["E_mag"]
        df["P_minus_L"] = df["P_mag"] - df["L_mag"]
        df["P_minus_E_k"] = df["P_minus_E"] / 1000.0
        df["P_minus_L_k"] = df["P_minus_L"] / 1000.0


def save_line(df, x, ys, labels, title, ylabel, filename):
    present = [(y, label) for y, label in zip(ys, labels) if y in df.columns]
    if not present:
        return

    plt.figure(figsize=(14, 6))
    for y, label in present:
        plt.plot(df[x], df[y], linewidth=0.8, label=label)

    plt.title(title)
    plt.xlabel("Epoch ms")
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(filename, dpi=150)
    plt.close()


def show_quad(df, x, filename=None, show=True):
    Ei = get_col(df, "Ei", "early_i", "E_i")
    Eq = get_col(df, "Eq", "early_q", "E_q")
    Pi = get_col(df, "Pi", "prompt_i", "P_i")
    Pq = get_col(df, "Pq", "prompt_q", "P_q")
    Li = get_col(df, "Li", "late_i", "L_i")
    Lq = get_col(df, "Lq", "late_q", "L_q")

    t = df[x]

    fig, ax = plt.subplots(4, 1, figsize=(18, 9), sharex=True)

    ax[0].plot(t, Ei * Ei, "bo-", markersize=2, linewidth=0.5, label="early")
    ax[0].plot(t, Pi * Pi, "ro-", markersize=2, linewidth=0.5, label="prompt")
    ax[0].plot(t, Li * Li, "g+-", markersize=3, linewidth=0.5, label="late")
    ax[0].set_title("I channel")
    ax[0].set_ylabel("I$^2$")
    ax[0].legend(loc="upper right")

    ax[1].plot(t, Eq * Eq, "co-", markersize=2, linewidth=0.5, label="early")
    ax[1].plot(t, Pq * Pq, "mo-", markersize=2, linewidth=0.5, label="prompt")
    ax[1].plot(t, Lq * Lq, "y+-", markersize=3, linewidth=0.5, label="late")
    ax[1].set_title("Q channel")
    ax[1].set_ylabel("Q$^2$")
    ax[1].legend(loc="upper right")

    ax[2].plot(t, Pi, "r+", markersize=4, linewidth=0.5)
    ax[2].set_title("Prompt I")
    ax[2].set_ylabel("I")

    ax[3].plot(t, Pq, "bo", markersize=3, fillstyle="none", linewidth=0.5)
    ax[3].set_title("Prompt Q")
    ax[3].set_ylabel("Q")
    ax[3].set_xlabel("ms")

    for a in ax:
        a.grid(True)

    plt.tight_layout()

    if filename is not None:
        plt.savefig(filename, dpi=150)

    if show:
        plt.show()
    else:
        plt.close()


def save_all(df, x, out_dir):
    first = df[x].iloc[0]
    zoom = df[(df[x] >= first) & (df[x] < first + 1000)].copy()

    save_line(
        df,
        x,
        ["E_mag_k", "P_mag_k", "L_mag_k"],
        ["Early", "Prompt", "Late"],
        "Early / Prompt / Late magnitude",
        "Magnitude (k)",
        out_dir / "20_epl_magnitude.png",
    )

    save_line(
        df,
        x,
        ["Ei_k", "Pi_k", "Li_k"],
        ["Early I", "Prompt I", "Late I"],
        "Early / Prompt / Late I",
        "I value (k)",
        out_dir / "21_epl_i.png",
    )

    save_line(
        df,
        x,
        ["Eq_k", "Pq_k", "Lq_k"],
        ["Early Q", "Prompt Q", "Late Q"],
        "Early / Prompt / Late Q",
        "Q value (k)",
        out_dir / "22_epl_q.png",
    )

    save_line(
        zoom,
        x,
        ["E_mag_k", "P_mag_k", "L_mag_k"],
        ["Early", "Prompt", "Late"],
        "Early / Prompt / Late magnitude, first 1000 ms",
        "Magnitude (k)",
        out_dir / "23_epl_magnitude_zoom_1000ms.png",
    )

    save_line(
        zoom,
        x,
        ["Ei_k", "Pi_k", "Li_k"],
        ["Early I", "Prompt I", "Late I"],
        "Early / Prompt / Late I, first 1000 ms",
        "I value (k)",
        out_dir / "24_epl_i_zoom_1000ms.png",
    )

    save_line(
        zoom,
        x,
        ["Eq_k", "Pq_k", "Lq_k"],
        ["Early Q", "Prompt Q", "Late Q"],
        "Early / Prompt / Late Q, first 1000 ms",
        "Q value (k)",
        out_dir / "25_epl_q_zoom_1000ms.png",
    )

    save_line(
        zoom,
        x,
        ["P_minus_E_k", "P_minus_L_k"],
        ["Prompt - Early", "Prompt - Late"],
        "Prompt dominance, first 1000 ms",
        "Magnitude delta (k)",
        out_dir / "26_prompt_dominance_zoom_1000ms.png",
    )

    save_line(
        zoom,
        x,
        ["dll_disc"],
        ["DLL discriminator"],
        "DLL discriminator, first 1000 ms",
        "DLL discriminator",
        out_dir / "27_dll_disc_zoom_1000ms.png",
    )

    save_line(
        zoom,
        x,
        ["pll_disc"],
        ["PLL discriminator"],
        "PLL discriminator, first 1000 ms",
        "PLL discriminator",
        out_dir / "28_pll_disc_zoom_1000ms.png",
    )

    save_line(
        zoom,
        x,
        ["code_phase"],
        ["Code phase"],
        "Code phase, first 1000 ms",
        "Code phase chips",
        out_dir / "29_code_phase_zoom_1000ms.png",
    )

    save_line(
        zoom,
        x,
        ["doppler"],
        ["Doppler"],
        "Doppler, first 1000 ms",
        "Doppler Hz",
        out_dir / "30_doppler_zoom_1000ms.png",
    )

    save_line(
        zoom,
        x,
        ["carrier_nco_hz"],
        ["Carrier NCO"],
        "Carrier NCO, first 1000 ms",
        "Frequency Hz",
        out_dir / "32_carrier_nco_zoom_1000ms.png",
    )

    save_line(
        zoom,
        x,
        ["code_nco_hz"],
        ["Code NCO"],
        "Code NCO, first 1000 ms",
        "Frequency Hz",
        out_dir / "33_code_nco_zoom_1000ms.png",
    )

    if {"Pi_k", "Pq_k", "Ei_k", "Eq_k", "Li_k", "Lq_k"}.issubset(df.columns):
        plt.figure(figsize=(8, 8))
        plt.scatter(df["Pi_k"], df["Pq_k"], s=2, label="Prompt")
        plt.scatter(df["Ei_k"], df["Eq_k"], s=2, label="Early")
        plt.scatter(df["Li_k"], df["Lq_k"], s=2, label="Late")
        plt.title("Early / Prompt / Late I/Q scatter")
        plt.xlabel("I (k)")
        plt.ylabel("Q (k)")
        plt.grid(True)
        plt.legend()
        plt.axis("equal")
        plt.tight_layout()
        plt.savefig(out_dir / "31_epl_iq_scatter.png", dpi=150)
        plt.close()

    if {"Pi", "Pq"}.issubset(df.columns):
        df["prompt_phase_rad"] = np.unwrap(np.arctan2(df["Pq"], df["Pi"]))
        df["prompt_phase_step_rad"] = df["prompt_phase_rad"].diff()
        df["prompt_residual_hz"] = df["prompt_phase_step_rad"] / (2.0 * np.pi * 0.001)

    save_line(
        df,
        x,
        ["prompt_phase_rad"],
        ["Prompt phase"],
        "Prompt phase from atan2(Pq, Pi)",
        "Radians",
        out_dir / "34_prompt_phase.png",
    )

    save_line(
        df,
        x,
        ["prompt_residual_hz"],
        ["Prompt residual"],
        "Estimated residual carrier from prompt phase",
        "Hz",
        out_dir / "35_prompt_residual_hz.png",
    )

    save_line(
        df,
        x,
        ["prompt_residual_hz", "doppler"],
        ["Prompt residual Hz", "Doppler Hz"],
        "Prompt residual carrier vs Doppler",
        "Hz",
        out_dir / "36_prompt_residual_vs_doppler.png",
    )

    save_line(
        df,
        x,
        ["prompt_residual_hz", "carrier_nco_hz"],
        ["Prompt residual Hz", "Carrier NCO Hz"],
        "Prompt residual carrier vs Carrier NCO",
        "Hz",
        out_dir / "37_prompt_residual_vs_carrier_nco.png",
    )  

    save_line(
        df,
        x,
        ["doppler", "carrier_nco_hz"],
        ["Doppler", "Carrier NCO"],
        "Doppler vs Carrier NCO, full run",
        "Hz",
        out_dir / "38_doppler_vs_carrier_nco_full.png",
    )

    save_line(
        df,
        x,
        ["pll_disc", "doppler"],
        ["PLL discriminator", "Doppler"],
        "PLL discriminator vs Doppler, full run",
        "Mixed units",
        out_dir / "39_pll_vs_doppler_full.png",
    )

    save_line(
        df,
        x,
        ["dll_disc", "code_phase"],
        ["DLL discriminator", "Code phase"],
        "DLL discriminator vs Code phase, full run",
        "Mixed units",
        out_dir / "40_dll_vs_code_phase_full.png",
    )

    save_line(
        df,
        x,
        ["is_locked"],
        ["Is locked"],
        "Receiver lock flag, full run",
        "Locked",
        out_dir / "41_is_locked_full.png",
    )   

def main():
    if len(sys.argv) < 2:
        print("usage: python tools/plot_tracking.py replay_tracking.csv [--save-only]")
        sys.exit(1)

    csv_path = Path(sys.argv[1])
    save_only = "--save-only" in sys.argv

    df = pd.read_csv(csv_path, sep=None, engine="python").copy()
    if df.empty:
        print(f"no rows found in {csv_path}")
        sys.exit(1)

    x = pick_x(df)
    normalize_columns(df)

    out_dir = csv_path.with_suffix("")
    out_dir.mkdir(exist_ok=True)

    show_quad(
        df,
        x,
        filename=out_dir / "00_tracking_quad.png",
        show=not save_only,
    )

    save_all(df, x, out_dir)

    print(f"wrote tracking plots to: {out_dir}")


if __name__ == "__main__":
    main()