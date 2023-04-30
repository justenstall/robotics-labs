import time


class Controller(object):
    """
    A class represenation of a PID controller
    """

    def __init__(self, name, max_errors=10, Kp=1, Ki=1, Kd=1, useDeltaT=False):
        """
        Constructor, sets up class
        """
        self.name = name
        self.max_errors = max_errors

        # Chase's gains: p=8, i=1, d=0
        # Chase's gains: p=.1, i=1, d=0
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.useDeltaT = useDeltaT

        self.error_array = [0] * self.max_errors

        self.n = 0

        # Initialize timing
        self.startTime = time.perf_counter()  # returns time in seconds
        self.elapsedTime = 0  # initialize elapsed time
        self.previousTime = self.startTime  # time of previous iteration

    # ------------------- Mode Control ------------------------

    # Runs the next iteration of the PID controller, ingesting error_n into the error tracking
    # Returns the output of the PID formula
    def iterate(self, error_n):
        # get the current time of this iteration
        iterationTime = time.perf_counter()

        # increase N
        self.n = self.n + 1

        # Store the current error
        self.error_array[self.index()] = error_n

        error_n_minus_1 = self.error(self.n - 1)

        proportional = self.Kp * error_n
        integral = self.Ki * sum(self.error_array)
        derivative = self.Kd * (error_n_minus_1 - error_n)
        if self.useDeltaT:
            derivative = derivative / self.deltaT()

        output = proportional + integral + derivative

        print(
            f"{self.name}[{self.n}]: error={error_n:5} --> out={output:5}\n{int(proportional)}P+{int(integral)}I+{int(derivative)}D"
        )

        self.previousTime = iterationTime  # update the iteration time

        return output

    def error(self, n=None):
        if n is None:
            n = self.n
        return self.error_array[self.index(n)]

    def index(self, n=None):
        if n is None:
            n = self.n
        return n % self.max_errors

    def deltaT(self, now: float):
        # difference between checkTime and startTime is how much time has passed
        self.elapsedTime = now - self.previousTime  # update elapsed time
