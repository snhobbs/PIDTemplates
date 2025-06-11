from dataclasses import dataclass, field
from collections import deque

def trapasoid_integral(p1, p2, h=1):
    area = ((p1+p2)*h)/2
    return area

def simc_tuning(delay: float, slope: float):
    alpha = 16
    beta = 0.4
    kp = beta / (2 * slope * delay)
    ti = alpha * delay
    return kp, ti

def zn_tuning(delay: float, slope: float):
    alpha = 0.714#tuning perameter Ziegler-Nichols
    beta = 3.33#tuning perameter Ziegler-Nichols
    kp = alpha/(slope*delay)
    ti = beta*delay
    return kp, ti

def inverse_response_tuning(delay: float, slope: float):
    c = 2.75
    ti = (2*c+1)*delay
    kp = (2*c+1)/(slope*self.delay*(c+1)**2)
    return kp, ti

def calculate_pi_coefficients(
        delay, gain, update_frequency=1, method="smic"):
    '''
    Update frequency is used to set the integrator gain correctly as it needs to be gain/unit time
    '''
    methods = ("smic", "zn", "inverse_response")
    if method == "smic":
        kp, ti = simc_tuning(delay=delay, slope=gain)
    elif method == "kn":
        kp, ti = zn_tuning(delay=delay, slope=gain)
    elif method == "inverse_response":
        kp, ti = inverse_response_tuning(delay=delay, slope=gain)
    else:
        raise ValueError(f"Method not one of {methods}")

    ki = (kp / ti) / update_frequency
    return kp, ki


@dataclass
class IIR_PI_Filter:
    kp: float
    ki: float
    sample_time: float = 0
    set_point: float = 0
    integral: float = 0
    control: float = 0
    primed: bool = False
    sample: float = 0  #  Sample index
    last_error: float = 0
    error: float = 0
    ilim: float = 0
    antiwindup: bool = False

    def update_(self, reading):
        error = reading - self.set_point
        #//assert(error + set_ == reading)

        self.last_error = self.error
        self.error = error

        kprime_length = 2 # Samples to setup filter

        #//  Give time to prime filter
        if (not self.primed):
            if (self.sample >= kprime_length):
                self.primed = True

            # Run just the proportional controller when not primed
            self.control = -(error*self.kp)

        else:
            self.integral += trapasoid_integral(self.error, self.last_error, self.sample_time)
            self.control = -(error*self.kp + self.integral*self.ki)
        self.sample += 1
        return self.control

    def update(self, reading):
        integral = self.integral
        set_point = self.update_(reading)

        if self.ilim > 0:
            current_limit = True
            if (set_point > self.ilim):
                set_point = self.ilim

            elif (set_point < -self.ilim):
                set_point = -self.ilim

            else:
                current_limit = False

            # dont update the integral if in current limit
            if self.antiwindup and current_limit:
                self.integral = integral
        return set_point
