import atexit
import os
import signal
import subprocess
from contextlib import contextmanager
from subprocess import Popen


def kill_process(proc: Popen):
    if proc.poll() is None:
        os.kill(-proc.pid, signal.SIGINT)
        proc.wait()


@contextmanager
def managed_process(*args, **kwargs):
    """
    A context managed process group that kills the process group when the context or
    when the script is exited. This avoid zombie processes in `ros2 run`, `ros2 launch` and other
    process launchers that do not kill their subprocesses on exit.

    :param args: The arguments to pass to `subprocess.Popen`.
    :param kwargs: The keyword arguments to pass to `subprocess.Popen`, except `start_new_session`
        which will always be `True`.
    """
    with subprocess.Popen(*args, **kwargs, start_new_session=True) as proc:
        exit_cb = lambda: kill_process(proc)
        atexit.register(exit_cb)
        try:
            yield proc
        finally:
            exit_cb()
            atexit.unregister(exit_cb)
