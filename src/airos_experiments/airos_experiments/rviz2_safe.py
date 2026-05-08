from __future__ import annotations

import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

from ament_index_python.packages import get_package_prefix


_child: subprocess.Popen[bytes] | None = None
_shutdown_deadline: float | None = None
_shutdown_requested = False


def _rviz2_executable() -> str:
    prefix = Path(get_package_prefix('rviz2'))
    candidate = prefix / 'lib' / 'rviz2' / 'rviz2'
    if candidate.exists():
        return str(candidate)

    fallback = shutil.which('rviz2')
    if fallback is not None:
        return fallback

    raise RuntimeError('rviz2 executable was not found')


def _request_shutdown(signum: int, _frame: object) -> None:
    del signum

    global _shutdown_deadline
    global _shutdown_requested

    _shutdown_requested = True
    _shutdown_deadline = time.monotonic() + 2.0

    if _child is None or _child.poll() is not None:
        return

    try:
        os.killpg(_child.pid, signal.SIGTERM)
    except ProcessLookupError:
        return


def main(argv: list[str] | None = None) -> int:
    global _child

    args = sys.argv[1:] if argv is None else argv
    signal.signal(signal.SIGINT, _request_shutdown)
    signal.signal(signal.SIGTERM, _request_shutdown)

    _child = subprocess.Popen(
        [_rviz2_executable(), *args],
        start_new_session=True,
    )

    while True:
        return_code = _child.poll()
        if return_code is not None:
            if _shutdown_requested:
                return 0
            return return_code if return_code >= 0 else 128 + abs(return_code)

        if (
            _shutdown_deadline is not None
            and time.monotonic() >= _shutdown_deadline
        ):
            try:
                os.killpg(_child.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            _child.wait()
            return 0

        time.sleep(0.1)


if __name__ == '__main__':
    raise SystemExit(main())
