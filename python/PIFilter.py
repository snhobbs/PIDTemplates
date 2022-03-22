def DelayIntegratorSmicTuning(
        delay_seconds, gain, update_frequency):
    alpha = 16
    beta = 0.4
    kp = beta / (2 * gain * delay_seconds)
    ti = alpha * delay_seconds
    ki = (kp / ti) / update_frequency
    return kp, ki

def DelayIntegratorFromSmicTuning(
        kp, ki, update_frequency):
    alpha = 16
    beta = 0.4
    ti = (kp/ki)/update_frequency
    delay_seconds = ti/alpha
    gain = beta / (2*kp*delay_seconds)
    return delay_seconds, gain

'''
/*
 * Trapasoidal integral (p1+p2)/2 * h
 * */
'''
def trapasoid_integral(p1, p2, h=1):
    area = ((p1+p2)*h)/2
    return area


class IIR_PI_Filter:
    def __init__(self, kp, ki, sample_time, set_point):
        self.kp_ = kp
        self.ki_ = ki
        self.sample_time_ = sample_time
        self.set_ = set_point

        self.integral_ = 0
        self.control_ = 0
        self.sample_ = 0
        self.errors_ = [0, 0]

    @property
    def kp(self):
        return self.kp_

    @kp.setter
    def kp(self, value):
        self.kp_ = value

    @property
    def integral(self):
        return self.integral_

    @property
    def ki(self):
        return self.ki_

    @ki.setter
    def ki(self, value):
        self.ki_ = value

    @property
    def set(self):
        return self.set_

    @set.setter
    def set(self, value):
        self.set_ = value

    @property
    def control(self):
        return self.control_

    @control.setter
    def control(self, value):
        self.control_ = value


    def update(self, reading):
        error = reading - self.set
        #//assert(error + set_ == reading)

        self.errors_[1] = self.errors_[0]
        self.errors_[0] = error

        #//  Give time to prime filter
        if (self.sample_ >= len(self.errors_)):
            self.integral_ += trapasoid_integral(self.errors_[0], self.errors_[1], self.sample_time_)

        self.control_ = -(error*self.kp + self.integral_*self.ki)
        self.sample_ += 1
        return self.control_
