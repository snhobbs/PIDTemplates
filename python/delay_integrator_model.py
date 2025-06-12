from dataclasses import dataclass, field
from collections import deque
from scipy import signal
import numpy as np
from PIFilter import calculate_pi_coefficients


def quadratic_calculate_heat_leak(temperature, ambient, factor):
    '''
    Convection Model
    acts like a control value in the direction of ambient temp
    '''
    delta = ambient - temperature
    return factor * delta * abs(delta)  # preserves sign


def linear_calculate_heat_leak(temperature, ambient, factor):
    '''
    Conduction Model
    acts like a control value in the direction of ambient temp
    '''
    return (ambient - temperature) * factor  # temp > ambient -> negative leak


class Plant:
    def __init__(self, delay, slope):
        self.delay = float(delay)  # delay time in seconds
        self.slope = float(slope)  # process/time/control

    def transfer(self, s):
        return self.slope*np.exp(-s*self.delay)/s


@dataclass
class System:
    plant: Plant
    update_rate: float = 1
    tuning_method: str = "simc"

    @property
    def slope(self):
        return self.plant.slope

    @property
    def delay(self):
        return self.plant.delay

    @property
    def kp(self):
        kp, _ = calculate_pi_coefficients(delay=self.delay, gain=self.slope, update_frequency=self.update_rate, method=self.tuning_method)
        return kp

    @property
    def ti(self):
        _, ti = calculate_pi_coefficients(delay=self.delay, gain=self.slope, update_frequency=self.update_rate, method=self.tuning_method)
        return ti

    @property
    def ki(self):
        return self.kp / self.ti

    def transfer_closed_loop(self, s):
        k = self.slope*self.kp
        ti = self.ti
        g_s = k*np.exp(-self.delay*s)*(ti*s + 1)/(ti*s**2)
        return g_s / (1 + g_s)

    def transfer_open_loop(self, s):
        k = self.slope * self.kp
        ti = self.ti
        return k * np.exp(-self.delay * s) * (ti * s + 1) / (ti * s**2)

    def make_bode(self, w=None):
        system = self.lti()
        omega, mag, phase = signal.bode(system, w=w)
        phase = [p - ((180 / np.pi) * w * self.delay) for p, w in zip(phase, omega)]
        return omega, mag, phase

    def phase_margin(self):
        omega, mag, phase = self.make_bode()
        freq = omega / (2 * np.pi)
        crossover = next((i for i, m in enumerate(mag) if m <= 0), None)
        if crossover is None:
            raise ValueError("No 0 dB crossover found.")
        pm = 180 + phase[crossover]
        return max(phase) + 180, pm, freq[crossover]

    def lti(self):
        k = self.slope*self.kp
        ti = self.ti
        return signal.lti([k*ti, k], [ti, 0, 0])  #  k*(s + ti)/(ti*s**2)


@dataclass
class DelayIntegratorPlantModel:
    delay: float
    gain: float
    ambient: float
    rate: float = 1
    heat_leak: float = 0
    temperature: float = 0
    history: deque = field(default_factory=deque)

    def reset(self):
        self.history.clear()
        self.temperature = self.ambient

    def update(self, control):
        time_step = 1. / self.rate
        self.history.appendleft(control)
        while len(self.history) > self.delay * self.rate:
            setting = self.history[-1]  #  front
            self.history.pop()
            temperature_forcing = linear_calculate_heat_leak(
                self.temperature, self.ambient, self.heat_leak
            )
            self.temperature += (setting + temperature_forcing) * time_step * self.gain
        return self.temperature
