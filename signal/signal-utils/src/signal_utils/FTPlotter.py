from matplotlib import pyplot as plt
import numpy as np


class FTPlotter:
    def plotPolar(self, x, t, X, f):
        # https://numpy.org/doc/stable/reference/generated/numpy.absolute.html
        XMag = np.abs(X)
        # https://numpy.org/doc/stable/reference/generated/numpy.angle.html
        XPhase = np.angle(X)
        # https://numpy.org/doc/stable/reference/generated/numpy.unwrap.html
        XPhaseUnwrapped = np.unwrap(XPhase)

        fig, axes = plt.subplots(1, 3, figsize=(10, 3))

        fig.suptitle("DFT (polar coordinates)")

        axes[0].plot(t, x)
        axes[0].set_title("Time domain")
        axes[0].set_xlabel("t")
        axes[0].set_ylabel("Amplitude")

        axes[1].plot(f, XMag)
        axes[1].set_xlabel("f")
        axes[1].set_ylabel("Magnitude")

        axes[2].plot(f, XPhase, label="Wrapped")
        axes[2].plot(f, XPhaseUnwrapped, color="r", label="Unwrapped")
        axes[2].set_xlabel("f")
        axes[2].set_ylabel("Phase (rad)")

        plt.tight_layout()

        # Draw to compute subplot positions, then place a centered title between the 2nd and 3rd axes
        fig.canvas.draw()
        pos1 = axes[1].get_position()
        pos2 = axes[2].get_position()
        xTitle = (pos1.x0 + pos1.x1 + pos2.x0 + pos2.x1) / 4
        yTitle = max(pos1.y1, pos2.y1) + 0.01
        fig.text(
            xTitle,
            yTitle,
            "Frequency domain",
            ha="center",
            va="bottom",
            fontsize=plt.rcParams["axes.titlesize"],
        )

        plt.legend()
        plt.show()

    def plotRect(self, x, t, X, f):
        # https://numpy.org/doc/stable/reference/generated/numpy.real.html
        XRe = np.real(X)
        # https://numpy.org/doc/stable/reference/generated/numpy.imag.html
        XIm = np.imag(X)

        fig, axes = plt.subplots(1, 3, figsize=(10, 3))

        fig.suptitle("DFT (rectangular coordinates)")

        axes[0].plot(t, x)
        axes[0].set_title("Time domain")
        axes[0].set_xlabel("t")
        axes[0].set_ylabel("Amplitude")

        axes[1].plot(f, XRe)
        axes[1].set_xlabel("f")
        axes[1].set_ylabel("Real part")

        axes[2].plot(f, XIm)
        axes[2].set_xlabel("f")
        axes[2].set_ylabel("Imaginary part")

        plt.tight_layout()

        # Draw to compute subplot positions, then place a centered title between the 2nd and 3rd axes
        fig.canvas.draw()
        pos1 = axes[1].get_position()
        pos2 = axes[2].get_position()
        xTitle = (pos1.x0 + pos1.x1 + pos2.x0 + pos2.x1) / 4
        yTitle = max(pos1.y1, pos2.y1) + 0.01
        fig.text(
            xTitle,
            yTitle,
            "Frequency domain",
            ha="center",
            va="bottom",
            fontsize=plt.rcParams["axes.titlesize"],
        )

        plt.show()
