from dataclasses import dataclass
import numbers

def trapezoid_integral(p1, p2, h=1):
    area = ((p1+p2)*h)/2
    return area

def simc_tuning(delay: float, slope: float, *, lam_factor: float = 3):
    """
    SIMC tuning for an integrating process with delay:
    G(s) = K / s * e^(-theta * s)

    Parameters:
        delay (float): Process delay (theta)
        slope (float): Ramp slope (R)
        lam_factor (float): Multiplier for desired closed-loop time constant λ (default λ = delay)

    Returns:
        kp (float): Proportional gain
        ti (float): Integral time
    """
    lam = lam_factor * delay
    ti = 4 * (delay + lam)
    kp = 1 / (slope * (delay + lam))
    return kp, ti

def calculate_pi_coefficients(
        delay, gain, update_frequency=1, method="simc", **kwargs):
    '''
    Update frequency is used to set the integrator gain correctly as it needs to be gain/unit time
    '''
    methods = ("simc",)
    if method == "simc":
        kp, ti = simc_tuning(delay=delay, slope=gain, **kwargs)
    else:
        raise ValueError(f"Method {method} not one of {methods}")

    ki = (kp / ti) / update_frequency
    return kp, ki


@dataclass
class IIR_PI_Filter:
    '''
    PI Filter including current limit and antiwindup.
    '''
    kp: float
    ki: float
    sample_time: float = 1
    set_point: float = 0
    integral: float = 0
    control: float = 0
    primed: bool = False
    sample: float = 0  #  Sample index
    last_error: float = 0
    error: float = 0
    ilim: float | None | tuple[float, float] = ()
    antiwindup: bool = False

    def reset(self):
        self.integral = 0
        self.primed=False
        self.error=0
        self.last_error=0
        self.sample=0
        self.control=0

    def update_(self, reading):
        error = reading - self.set_point
        self.last_error = self.error
        self.error = error

        kprime_length = 2 # Samples to setup filter

        control = 0
        # Give time to prime filter. Runs only the proportional component at first.
        if (not self.primed):
            if (self.sample >= kprime_length):
                self.primed = True

            # Run just the proportional controller when not primed
            control = -(error*self.kp)

        else:
            self.integral += trapezoid_integral(self.error, self.last_error, self.sample_time)
            control = -(error*self.kp + self.integral*self.ki)
        self.sample += 1
        self.control = control
        return self.control

    def update(self, reading):
        last_integral = self.integral
        control = self.update_(reading)

        if self.ilim:
            n_ilim, p_ilim = (-self.ilim, self.ilim) if isinstance(self.ilim, numbers.Number) else (min(self.ilim), max(self.ilim))
            limit_exceeded = False
            if (control > p_ilim):
                control = p_ilim
                limit_exceeded = True

            elif (control < n_ilim):
                control = n_ilim
                limit_exceeded = True

            # if in current limit use the previous integrator value
            if self.antiwindup and limit_exceeded:
                self.integral = last_integral

        self.control = control
        return self.control
