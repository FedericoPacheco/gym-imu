from array import array
from signal_utils import IMUSampleReader
import sys

SAMPLING_FREQUENCY = 30  # In Hz
SAMPLING_PERIOD = 1.0 * 1000 * 1000 / SAMPLING_FREQUENCY  # T = 1/f, in microseconds


def fix(inputFilePath: str, samples: dict[str, array | dict[str, array]]) -> None:

    print(f"File about to be fixed: {inputFilePath}")

    outputFileName = f"fixed-{str.split(inputFilePath, '/')[ -1]}"
    tPrev = samples["t"][0]  # type: ignore
    outputIdx = 0
    with open(outputFileName, "w", encoding="utf-8") as f:
        f.write("seq,ax,ay,az,roll,pitch,yaw\n")
        for idx in range(len(samples["t"])):  # type: ignore
            diff = samples["t"][idx] - tPrev  # type: ignore
            tPrev = samples["t"][idx]  # type: ignore
            if (
                abs(diff) > 1.68 * SAMPLING_PERIOD
            ):  # More than one std deviation away from the mean
                print(
                    f"Sample number: {outputIdx + 1} may have been lost, inter-sample timestamp diff: {diff} us"
                )
                # As seen on histogram of diffs, at most 1 sample at a time could have been lost in a small minority of cases.
                # Alternatively, in many cases it seems to be a link between transmission batch size and delayed timestamps on the next sample of the next batch
                outputIdx += 2
            else:
                outputIdx += 1

            f.write(f"{outputIdx},{samples['a']['x'][idx]},{samples['a']['y'][idx]},{samples['a']['z'][idx]},{samples['w']['roll'][idx]},{samples['w']['pitch'][idx]},{samples['w']['yaw'][idx]}\n")  # type: ignore

    print(f"File with fixed timestamps saved to ./{outputFileName}")


# Run: python fixTimestamps.py <inputFilePath>
if __name__ == "__main__":
    reader = IMUSampleReader()

    if len(sys.argv) > 1:
        inputFilePath = sys.argv[1]
        samples = reader.read(inputFilePath)
        fix(inputFilePath, samples)
    else:
        samples = reader.read("File name not provided!")
