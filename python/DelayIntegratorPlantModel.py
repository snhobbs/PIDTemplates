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

class DelayIntegratorPlantModel:
    def __init__(self, delay, gain, rate, ambient, heat_leak):
        self.delay_ = delay
        self.gain_ = gain
        self.rate_ = rate
        self.ambient_ = ambient
        self.heat_leak_ = heat_leak
        self.temperature_ = self.ambient_
        self.history_ = deque()

    @property
    def ambient(self):
        return self.ambient_

    @ambient.setter
    def ambient(self, ambient):
        self.ambient_ = ambient

    @property
    def temperature(self):
        return self.temperature_

    @temperature.setter
    def temperature(self, temperature):
        self.temperature_ = temperature


    def reset(self):
        self.history_.clear()
        self.temperature = self.ambient_

    def update(self, control):
        time_step = 1./self.rate_
        self.history_.appendleft(control)
        while len(self.history_) > self.delay_*self.rate_:
            setting = self.history_[-1]  #  front
            self.history_.pop()
            temperature_forcing = linear_calculate_heat_leak(self.temperature, self.ambient_, self.heat_leak_)
            self.temperature += (setting + temperature_forcing) * time_step * self.gain_
        #//  amps * dt * dT/dt/amps
        #//  temp/s/control, use last control setting as there is a delay
        return self.temperature
