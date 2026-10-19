"""Explicit, bounded read-only access to the AS2 C++ GetState probe."""
import json
import os
import re
import shutil
import subprocess
import threading
from datetime import datetime, timezone
from pathlib import Path


class StateReadError(ValueError):
    pass


def decode_state(raw):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise StateReadError('Duplicate state field')
            result[key] = value
        return result

    if len(raw) > 65536:
        raise StateReadError('State response too large')
    try:
        value = json.loads(raw, object_pairs_hook=unique)
    except (ValueError, TypeError) as exc:
        raise StateReadError('Probe returned invalid JSON') from exc
    if not isinstance(value, dict) or set(value) != {'sdk_return_code', 'state'}:
        raise StateReadError('Invalid probe response fields')
    if type(value['sdk_return_code']) is not int:
        raise StateReadError('Invalid SDK return code')
    if value['sdk_return_code'] != 0:
        raise StateReadError(f"SDK GetState failed: {value['sdk_return_code']}")
    state = value['state']
    if (not isinstance(state, dict) or not state or len(state) > 100 or
            any(not isinstance(v, str) or len(v) > 4096 for v in state.values())):
        raise StateReadError('Invalid state map')
    return state


class ReadOnlyAS2Bridge:
    def __init__(self, interface='eth1', distro='Ubuntu-22.04', probe=None):
        if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.:-]{0,31}', interface):
            raise StateReadError('Invalid network interface')
        if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,63}', distro):
            raise StateReadError('Invalid WSL distribution')
        self.interface, self.distro = interface, distro
        self.probe = Path(probe) if probe else Path(__file__).resolve().parents[1] / 'build/as2-probe/as2_state_probe'
        self.lock = threading.Lock()

    def command(self):
        path = self.probe.resolve()
        if not path.is_file():
            raise StateReadError('State probe is not built; run deploy/build_state_probe.sh in Linux')
        if os.name == 'nt':
            wsl = shutil.which('wsl.exe')
            if not wsl or not re.fullmatch(r'[A-Za-z]:', path.drive):
                raise StateReadError('WSL or a supported local probe path is unavailable')
            linux_path = '/mnt/' + path.drive[0].lower() + path.as_posix()[2:]
            return [wsl, '-d', self.distro, '--', 'timeout', '12', linux_path, self.interface, '--json']
        return ['timeout', '12', str(path), self.interface, '--json']

    def get_state(self):
        if not self.lock.acquire(blocking=False):
            raise StateReadError('A state request is already in progress')
        try:
            try:
                result = subprocess.run(self.command(), capture_output=True, text=True,
                                        encoding='utf-8', errors='replace', timeout=18, shell=False)
            except subprocess.TimeoutExpired as exc:
                raise StateReadError('State probe timed out') from exc
            except OSError as exc:
                raise StateReadError('Could not launch state probe') from exc
            state = decode_state(result.stdout)
            if result.returncode != 0:
                raise StateReadError('State probe exited unsuccessfully')
            return {'backend': 'as2_sdk_read_only', 'hardware_connected': True,
                    'motion_enabled': False, 'observed_at': datetime.now(timezone.utc).isoformat(),
                    'interface': self.interface, 'state': state, 'sdk_return_code': 0,
                    'source': 'SportClient.GetState', 'pose_available': False}
        finally:
            self.lock.release()

    def stop(self):
        raise StateReadError('Motion commands are not available on the read-only bridge')
