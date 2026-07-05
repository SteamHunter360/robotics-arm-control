import matplotlib.pyplot as plt


def plot_pid_tuning_comparison(results, save_path=None):
    names = list(results.keys())

    rms_errors = [results[name]["metrics"]["rms_error"] for name in names]
    overshoots = [results[name]["metrics"]["overshoot"] for name in names]
    settling_times = [
        results[name]["metrics"]["settling_time"]
        if results[name]["metrics"]["settling_time"] is not None
        else 0.0
        for name in names
    ]
    control_efforts = [
        results[name]["metrics"]["rms_control_effort"] for name in names
    ]

    fig, ax = plt.subplots(figsize=(10, 6))

    x = range(len(names))

    ax.plot(x, rms_errors, marker="o", label="RMS Error")
    ax.plot(x, overshoots, marker="o", label="Overshoot")
    ax.plot(x, settling_times, marker="o", label="Settling Time")
    ax.plot(x, control_efforts, marker="o", label="RMS Control Effort")

    ax.set_xticks(list(x))
    ax.set_xticklabels(names)
    ax.set_title("PID Tuning Performance Comparison")
    ax.set_ylabel("Metric Value")
    ax.grid(True)
    ax.legend()

    fig.tight_layout()

    if save_path is not None:
        fig.savefig(save_path, dpi=300, bbox_inches="tight")

    plt.show()

    return fig