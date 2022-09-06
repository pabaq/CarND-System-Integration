MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        """ PID controller.

        Args:
            kp: proportional gain
            ki: integral gain
            kd: derivative gain
            mn: minimum allowed correction
            mx: maximum allowed correction
        """

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.mn = mn
        self.mx = mx

        # Intialize the integral term
        self.int_val = self.last_error = 0.

    def reset(self):
        """ Reset the PID controller. """
        self.int_val = self.last_error = 0.

    def step(self, error, time_step):
        """ Evaluate the next step.

        Args:
            error (float):
                the current error
            time_step (float):
                the passed time since the last evaluation step [s]

        Returns:
            the correction value due to the cumulated error
        """

        integral = self.int_val + error * time_step
        derivative = (error - self.last_error) / time_step

        correction = self.kp * error + self.ki * integral + self.kd * derivative

        if correction > self.mx:
            correction = self.mx
        elif correction < self.mn:
            correction = self.mn
        else:
            self.int_val = integral

        self.last_error = error

        return correction
