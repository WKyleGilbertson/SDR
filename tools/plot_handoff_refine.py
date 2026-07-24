import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("handoff_refine.csv")

for col in ["E", "P", "L"]:
    df[col + "_norm"] = df[col] / df[col].max()

best = df.loc[df["P"].idxmax()]

dll = df["dll"].values
codes = df["test_code"].values
best_code = best["test_code"]

zc = None
best_dist = None

for i in range(len(dll) - 1):
    crossed = (dll[i] >= 0 and dll[i + 1] <= 0) or (dll[i] <= 0 and dll[i + 1] >= 0)
    if crossed and dll[i] != dll[i + 1]:
        t = dll[i] / (dll[i] - dll[i + 1])
        candidate = codes[i] + t * (codes[i + 1] - codes[i])
        dist = abs(candidate - best_code)

        if best_dist is None or dist < best_dist:
            zc = candidate
            best_dist = dist

plt.figure(figsize=(11, 6))

plt.plot(df["test_code"], df["E_norm"], label="Early")
plt.plot(df["test_code"], df["P_norm"], label="Prompt")
plt.plot(df["test_code"], df["L_norm"], label="Late")
plt.plot(df["test_code"], df["dll"], label="DLL")

plt.axvline(df["input_code"].iloc[0], linestyle="--", label="Input code")
plt.axvline(best_code, linestyle=":", label=f"Best P {best_code:.4f}")

plt.plot(
    best_code,
    best["P_norm"],
    marker="o",
    markersize=8,
    color="black",
    label="Best P sample",
)

if zc is not None:
    plt.axvline(
        zc,
        color="black",
        linestyle="-.",
        linewidth=2,
        label=f"DLL zero {zc:.4f}",
    )

plt.title(f"Tracker handoff refinement PRN {int(df['prn'].iloc[0])}")
plt.xlabel("Tracker code phase (chips)")
plt.ylabel("Normalized magnitude / DLL")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

print("Best row:")
print(best)

if zc is not None:
    print(f"DLL zero crossing nearest Best P: {zc:.6f}")
    print(f"Best P - DLL zero: {best_code - zc:.6f} chips")
else:
    print("No DLL zero crossing found.")