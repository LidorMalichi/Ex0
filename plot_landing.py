import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# Load data
df = pd.read_csv("landing_report.csv")

# Apply clean visual style
sns.set_theme(style="whitegrid")

# Create custom layout for 5 plots - using gridspec for more control
fig = plt.figure(figsize=(16, 12))
gs = fig.add_gridspec(3, 2)

# Create axes with custom layout
ax1 = fig.add_subplot(gs[0, 0])  # Top left
ax2 = fig.add_subplot(gs[0, 1])  # Top right
ax3 = fig.add_subplot(gs[1, 0])  # Middle left
ax4 = fig.add_subplot(gs[1, 1])  # Middle right
ax5 = fig.add_subplot(gs[2, :])  # Bottom row, spanning both columns

# Plot Altitude
ax1.plot(df['time'], df['alt'], color='royalblue', linewidth=2)
ax1.set_title("Altitude vs Time", fontsize=14)
ax1.set_ylabel("Altitude (m)", fontsize=12)
ax1.set_xlabel("Time (s)", fontsize=12)

# Plot Vertical Speed
ax2.plot(df['time'], df['vs'], color='crimson', linewidth=2)
ax2.set_title("Vertical Speed vs Time", fontsize=14)
ax2.set_ylabel("Vertical Speed (m/s)", fontsize=12)
ax2.set_xlabel("Time (s)", fontsize=12)

# Plot Horizontal Speed
ax3.plot(df['time'], df['hs'], color='darkgreen', linewidth=2)
ax3.set_title("Horizontal Speed vs Time", fontsize=14)
ax3.set_ylabel("Horizontal Speed (m/s)", fontsize=12)
ax3.set_xlabel("Time (s)", fontsize=12)

# Plot Angle
ax4.plot(df['time'], df['ang'], color='mediumpurple', linewidth=2)
ax4.set_title("Angle vs Time", fontsize=14)
ax4.set_ylabel("Angle (deg)", fontsize=12)
ax4.set_xlabel("Time (s)", fontsize=12)

# Plot Fuel (wider plot in the bottom row)
ax5.plot(df['time'], df['fuel'], color='orange', linewidth=2)
ax5.set_title("Fuel vs Time", fontsize=14)
ax5.set_ylabel("Fuel (kg)", fontsize=12)
ax5.set_xlabel("Time (s)", fontsize=12)

# Adjust layout to avoid overlap and make the plot look balanced
plt.tight_layout(pad=3.0)

# Add space for the title
fig.subplots_adjust(top=0.92)

# Overall title
fig.suptitle("Bereshit Landing Simulation – Key Metrics", fontsize=18, fontweight='bold')

# Save and show the plot
plt.savefig("bereshit_landing.png", dpi=300)
plt.show()