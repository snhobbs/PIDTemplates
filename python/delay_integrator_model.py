from dataclasses import dataclass, field
from collections import deque

'''
def QuadraticCalculateHeatLeak(self, temperature, ambient, factor):
#//  acts like a control value in the direction of ambient temp
temp_diff = (ambient - temperature)
heat_leak = temp_diff * temp_diff * factor
return heat_leak * (temperature > ambient ? -1 : 1)
'''

def linear_calculate_heat_leak(temperature, ambient, factor):
    #//  acts like a control value in the direction of ambient temp
    return (ambient - temperature) * factor#  //  temp > ambient -> negative leak


class Plant(object):
    def __init__(self, delay, slope):
        self.delay = float(delay)#delay time in seconds
        self.slope = float(slope)#process/time/control

    def transfer(self, s):
        return self.slope*exp(-s*self.delay)/s


class System:
    def __init__(self, plant, update_rate):
        self.plant = plant
        self.update_rate = update_rate
        self.slope = plant.slope
        self.delay = plant.delay
        assert self.delay > 0
        self.kp, self.ti = simc_tuning(delay=self.delay, slope=self.slope)
        assert self.ti > 0
        self.ki = self.kp / self.ti

    def transfer_closed_loop(self, s):
        k = self.slope*self.kp
        ti = self.ti
        G_s = k*exp(-self.delay*s)*(ti*s + 1)/(ti*s**2)
        return G_s/(1+G_s)

    def transfer_open_loop(self, s):
        k = self.slope * self.kp
        ti = self.ti
        return k * np.exp(-self.delay * s) * (ti * s + 1) / (ti * s**2)

    def make_bode(self, w=None):
        '''
        system is a scipy LTI (
        '''
        system = self.lti()
        omega, mag, phase = signal.bode(system, w=w)
        phase = [p - ((180 / pi) * w * self.delay) for p, w in zip(phase, omega)]
        return omega, mag, phase

    def phase_margin(self):
        omega, mag, phase = self.make_bode()
        freq = omega / (2 * pi)
        crossover = next((i for i, m in enumerate(mag) if m <= 0), None)
        if crossover is None:
            raise ValueError("No 0 dB crossover found.")
        pm = 180 + phase[crossover]
        return max(phase) + 180, pm, freq[crossover]

    def lti(self):
        k = self.slope*self.kp
        ti = self.ti
        return signal.lti([k*ti, k], [ti, 0, 0])#k*(s + ti)/(ti*s**2)


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
        time_step = 1./self.rate
        self.history.appendleft(control)
        while len(self.history) > self.delay*self.rate:
            setting = self.history[-1]  #  front
            self.history.pop()
            temperature_forcing = linear_calculate_heat_leak(self.temperature, self.ambient, self.heat_leak)
            self.temperature += (setting + temperature_forcing) * time_step * self.gain
        #//  amps * dt * dT/dt/amps
        #//  temp/s/control, use last control setting as there is a delay
        return self.temperature
