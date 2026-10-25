"""Scaled monotonic clock so the mock robot can run faster than real time in tests."""
import asyncio
import time


class Clock:
    def __init__(self, speedup=1.0):
        if not 0.05 <= speedup <= 100:
            raise ValueError("speedup must be within [0.05, 100]")
        self.speedup = float(speedup)
        self._origin = time.monotonic()

    def now(self):
        """Seconds of (simulated) time since the clock was created."""
        return (time.monotonic() - self._origin) * self.speedup

    async def sleep(self, seconds):
        await asyncio.sleep(max(0.0, seconds) / self.speedup)

    def sleep_sync(self, seconds):
        time.sleep(max(0.0, seconds) / self.speedup)
