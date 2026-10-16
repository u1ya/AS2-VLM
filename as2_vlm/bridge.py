from typing import Protocol


class RobotBridge(Protocol):
    def get_state(self) -> dict: ...
    def stop(self) -> None: ...


class MockBridge:
    def __init__(self):
        self.stop_count = 0

    def get_state(self):
        return {"backend": "mock", "motion_enabled": False, "hardware_connected": False}

    def stop(self):
        self.stop_count += 1
