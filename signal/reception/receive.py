import asyncio
import matplotlib.pyplot as plt

from signal_utils import IMUSampleReceiver
from signal_utils import IMUSampleTimeSeriesPlotter


async def main() -> None:
    receiver = IMUSampleReceiver(listenDurationSeconds=30.0)
    await receiver.connect()
    motionData = await receiver.receive(exportedFileName="motionData.csv")
    await receiver.disconnect()

    timeSeriesPlotter = IMUSampleTimeSeriesPlotter()
    timeSeriesPlotter.plot(motionData)


asyncio.run(main())
