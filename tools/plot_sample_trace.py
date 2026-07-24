import sys
import pandas as pd
import matplotlib.pyplot as plt

if len(sys.argv) != 2:
    print("usage: python tools/plot_sample_trace.py sample_trace.csv")
    sys.exit(1)

df = pd.read_csv(sys.argv[1])

x = df["sample"]

def save_plot(cols, title, ylabel, filename):
    plt.figure(figsize=(14, 5))
    for col in cols:
        if col in df.columns:
            plt.plot(x, df[col], marker=".", linewidth=0.8, label=col)
    plt.title(title)
    plt.xlabel("sample")
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(filename, dpi=150)
    plt.close()

save_plot(
    ["early", "prompt", "late"],
    "Code taps",
    "tap value",
    "sample_trace_taps.png",
)

save_plot(
    ["raw_i", "bb_i", "bb_q"],
    "Raw and carrier-wiped samples",
    "sample value",
    "sample_trace_mixer.png",
)

save_plot(
    ["prompt_i_term", "prompt_q_term"],
    "Prompt contribution per sample",
    "term",
    "sample_trace_prompt_terms.png",
)

save_plot(
    ["Pi_running", "Pq_running"],
    "Prompt running accumulators",
    "accumulator",
    "sample_trace_prompt_running.png",
)

print("wrote sample_trace_*.png")