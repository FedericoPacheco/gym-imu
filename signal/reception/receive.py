import matplotlib.pyplot as plt

EXPORTED_DATA_FILE_NAME = "motionData"

motionData = [
    {
        "a": {"x": 0, "y": 0, "z": 0},
        "w": {"roll": 0, "pitch": 0, "yaw": 0},
        "t": 0,
    },
    {
        "a": {"x": 1, "y": 2, "z": 3},
        "w": {"roll": 45, "pitch": 90, "yaw": 180},
        "t": 1,
    },
    {
        "a": {"x": 2, "y": 4, "z": 6},
        "w": {"roll": 90, "pitch": 180, "yaw": 360},
        "t": 2,
    },
]


def plot():
    t = [row["t"] for row in motionData]
    ax = [row["a"]["x"] for row in motionData]
    ay = [row["a"]["y"] for row in motionData]
    az = [row["a"]["z"] for row in motionData]
    roll = [row["w"]["roll"] for row in motionData]
    pitch = [row["w"]["pitch"] for row in motionData]
    yaw = [row["w"]["yaw"] for row in motionData]

    fig, axes = plt.subplots(3, 2, figsize=(12, 8))

    axes[0, 0].plot(t, ax)
    axes[0, 0].set_ylabel("x (m/s²)")
    axes[0, 0].set_xlabel("time (s)")
    axes[0, 0].set_title("Linear Acceleration")

    axes[0, 1].plot(t, roll)
    axes[0, 1].set_ylabel("roll (deg/s)")
    axes[0, 1].set_xlabel("time (s)")
    axes[0, 1].set_title("Angular Velocity")

    axes[1, 0].plot(t, ay)
    axes[1, 0].set_ylabel("y (m/s²)")
    axes[1, 0].set_xlabel("time (s)")

    axes[1, 1].plot(t, pitch)
    axes[1, 1].set_ylabel("pitch (deg/s)")
    axes[1, 1].set_xlabel("time (s)")

    axes[2, 0].plot(t, az)
    axes[2, 0].set_ylabel("z (m/s²)")
    axes[2, 0].set_xlabel("time (s)")

    axes[2, 1].plot(t, yaw)
    axes[2, 1].set_ylabel("yaw (deg/s)")
    axes[2, 1].set_xlabel("time (s)")

    fig.suptitle("IMU Recorded Data")
    plt.tight_layout()
    fig.savefig("imuPlot.png")
    plt.show()
    print("Plot saved as imuPlot.png")


def exportMotionData():
    with open(f"{EXPORTED_DATA_FILE_NAME}.csv", "w") as f:
        f.write("t,ax,ay,az,roll,pitch,yaw\n")
        for row in motionData:
            f.write(
                f"{row['t']},{row['a']['x']},{row['a']['y']},{row['a']['z']},"
                f"{row['w']['roll']},{row['w']['pitch']},{row['w']['yaw']}\n"
            )
    print(f"Motion data saved to {EXPORTED_DATA_FILE_NAME}.csv")


plot()
exportMotionData()
