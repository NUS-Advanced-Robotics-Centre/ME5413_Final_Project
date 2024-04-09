import csv
import math
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.gridspec import GridSpec
import argparse

def evaluate_track_errors(errors_file):
    position_errors = []
    heading_errors = []
    relative_position_errors = []
    relative_heading_errors = []
    times = []

    with open(errors_file, 'r') as f:
        reader = csv.reader(f)
        for i, row in enumerate(reader):
            if len(row) < 4:  # Ensure there are enough columns in the row
                continue
            # Replace 'nan' with 0.0 before appending
            position_errors.append(float(row[0]) if not math.isnan(float(row[0])) else 0.0)
            heading_errors.append(float(row[1]) if not math.isnan(float(row[1])) else 0.0)
            relative_position_errors.append(float(row[2]) if not math.isnan(float(row[2])) else 0.0)
            relative_heading_errors.append(float(row[3]) if not math.isnan(float(row[3])) else 0.0)
            times.append(i)

    def rmse(errors):
        return math.sqrt(sum(err**2 for err in errors) / len(errors))

    rmse_position = rmse(position_errors)
    rmse_heading = rmse(heading_errors)
    rms_relative_position = rmse(relative_position_errors)
    rms_relative_heading = rmse(relative_heading_errors)

    print(f"RMSE Position: {rmse_position:.3f}")
    print(f"RMSE Heading: {rmse_heading:.3f}")
    print(f"RMS Relative Position: {rms_relative_position:.3f}")
    print(f"RMS Relative Heading: {rms_relative_heading:.3f}")

    fig = plt.figure(figsize=(10, 10))
    gs = GridSpec(4, 1, height_ratios=[1, 1, 1, 1])

    ax1 = fig.add_subplot(gs[0])
    ax2 = fig.add_subplot(gs[1])
    ax3 = fig.add_subplot(gs[2])
    ax4 = fig.add_subplot(gs[3])

    # Customize the appearance of each subplot
    for ax in [ax1, ax2, ax3, ax4]:
        ax.grid(True, linestyle='--', alpha=0.7)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.tick_params(labelsize=10)

    # Plot position error
    ax1.plot(times, position_errors, linewidth=1.5, color='#1f77b4')
    ax1.set_title(f"Position Error (RMSE: {rmse_position:.3f})", fontsize=12)
    ax1.set_ylabel("Error (m)", fontsize=10)
    ax1.xaxis.set_tick_params(labelbottom=False)

    # Plot heading error
    ax2.plot(times, heading_errors, linewidth=1.5, color='#ff7f0e')
    ax2.set_title(f"Heading Error (RMSE: {rmse_heading:.3f})", fontsize=12)
    ax2.set_ylabel("Error (rad)", fontsize=10)
    ax2.xaxis.set_tick_params(labelbottom=False)

    # Plot relative position error
    ax3.plot(times, relative_position_errors, linewidth=1.5, color='#2ca02c')
    ax3.set_title(f"Relative Position Error (RMS: {rms_relative_position:.3f})", fontsize=12)
    ax3.set_ylabel("Error (m)", fontsize=10)
    ax3.xaxis.set_tick_params(labelbottom=False)

    # Plot relative heading error
    ax4.plot(times, relative_heading_errors, linewidth=1.5, color='#d62728')
    ax4.set_title(f"Relative Heading Error (RMS: {rms_relative_heading:.3f})", fontsize=12)
    ax4.set_xlabel("Time", fontsize=10)
    ax4.set_ylabel("Error (rad)", fontsize=10)

    # Set y-axis tick format for all plots
    for ax in [ax1, ax2, ax3, ax4]:
        ax.yaxis.set_major_formatter(ticker.FormatStrFormatter('%.2f'))

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':


    file_path = "teb/teb_errors.csv"
    #
    # file_path = "dwa/dwa_errors.csv"

    evaluate_track_errors(file_path)
