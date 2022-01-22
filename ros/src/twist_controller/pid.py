
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx

        self.int_val = self.last_error = 0.

    def reset(self):
        """ Reset the PID controller. """
        self.int_val = self.last_error = 0.

    def step(self, error, time_step):
        """ Evaluate the next step.

        Parameters
        ----------
        error: float
            the current error
        time_step: float
            the passed time since the last evaluation step [s]
        """

        integral = self.int_val + error * time_step
        derivative = (error - self.last_error) / time_step

        val = self.kp * error + self.ki * integral + self.kd * derivative

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error

        return val
