import matplotlib.pyplot as plt

# Initialize lists to hold the data for the three curves
roll = []
pitch = []
yaw = []

# Open the file and read the data
with open("dvllog.txt", "r") as f:
    for line in f.readlines()[1:]:
        data = line.split(",")
        new_roll = float(data[0])
        new_pitch = float(data[1])
        new_yaw = float(data[2])
        if len(roll) > 0:
            prev_roll = roll[-1]
            prev_pitch = pitch[-1]
            prev_yaw = yaw[-1]
            if abs(new_roll - prev_roll) > 180:
                if new_roll > prev_roll:
                    new_roll -= 360
                else:
                    new_roll += 360
            if abs(new_pitch - prev_pitch) > 180:
                if new_pitch > prev_pitch:
                    new_pitch -= 360
                else:
                    new_pitch += 360
            if abs(new_yaw - prev_yaw) > 180:
                if new_yaw > prev_yaw:
                    new_yaw -= 360
                else:
                    new_yaw += 360

        roll.append(new_roll)
        pitch.append(new_pitch)
        yaw.append(new_yaw)

# Create the plot
plt.plot([60 * x / len(roll) for x in range(len(roll))], roll, label="Roll")
plt.plot([60 * x / len(pitch) for x in range(len(pitch))], pitch, label="Pitch")
plt.plot([60 * x / len(yaw) for x in range(len(yaw))], yaw, label="Yaw")

# Add a legend
plt.legend()

# Show the plot
plt.show()
