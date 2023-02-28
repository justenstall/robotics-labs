#Copied from Isidore Python Threading Examples

from threading import Timer, Lock
import time

class Periodic():
    """
    A periodic task running in threading.Timers
    """

    def __init__(self, interval, function, *args, **kwargs):
        self._lock = Lock()
        self._timer = None
        self.function = function
        self.interval = interval
        self.args = args
        self.kwargs = kwargs
        self._stopped = True
        self.next_call = None
        if kwargs.pop('autostart', True):
            self.start()

    def start(self, from_run=False):
        self._lock.acquire()

        if from_run or self._stopped:
            if not self.next_call:
                self.next_call = time.time()
            self.next_call += self.interval
            self._timer = Timer(self.next_call - time.time(), self._run)
            self._stopped = False
            self._timer.start()
            self._lock.release()

    def _run(self):
        self.start(from_run=True)
        self.function(*self.args, **self.kwargs)

    def stop(self):
        self._lock.acquire()
        self._stopped = True
        self._timer.cancel()
        self.next_call = None
        self._lock.release()