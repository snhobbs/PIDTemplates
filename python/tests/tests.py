import unittest
import sys
sys.path.append("../")
from PIFilter import *

class TestPIFilter(unittest.TestCase):
    def test_simc_tuning(self):
        kp, ti = simc_tuning(delay=1, slope=2)
        self.assertAlmostEqual(kp, 0.1)
        self.assertAlmostEqual(ti, 16)

    def test_zn_tuning(self):
        kp, ti = zn_tuning(delay=1, slope=2)
        self.assertAlmostEqual(kp, 0.357)
        self.assertAlmostEqual(ti, 3.33)

    def test_inverse_response_tuning(self):
        kp, ti = inverse_response_tuning(delay=1, slope=2)
        self.assertAlmostEqual(kp, (2 * 2.75 + 1) / (2 * (2.75 + 1) ** 2))
        self.assertAlmostEqual(ti, 6.5)

    def test_calculate_pi_coefficients_simc(self):
        kp, ki = calculate_pi_coefficients(1, 2, update_frequency=2, method="simc")
        self.assertAlmostEqual(kp, 0.1)
        self.assertAlmostEqual(ki, 0.1 / 16 / 2)

    def test_pi_response_basic(self):
        filter = IIR_PI_Filter(kp=0.5, ki=0.1)
        filter.set_point = 1.0
        output = filter.update(0.0)
        self.assertAlmostEqual(output, 0.5)

    def test_pi_response_saturates(self):
        ilim = 5
        filter = IIR_PI_Filter(kp=10, ki=10, ilim=ilim, antiwindup=True)
        filter.set_point = 10.0
        for _ in range(10):
            output = filter.update(0.0)
        self.assertLessEqual(abs(output), ilim)

    def test_zero_error_zero_output(self):
        f = IIR_PI_Filter(kp=1.0, ki=1.0)
        f.set_point = 5.0
        # Prime the filter
        f.update(0.0)
        f.update(0.0)
        output = f.update(5.0)
        self.assertAlmostEqual(output, 0.0, places=5)

    def test_integrator_accumulates(self):
        f = IIR_PI_Filter(kp=0.0, ki=1.0)
        f.set_point = 1.0
        # Prime the filter
        f.update(0.0)
        f.update(0.0)
        outputs = [f.update(0.0) for _ in range(5)]
        self.assertTrue(all(outputs[i] < outputs[i+1] for i in range(4)))

    def test_antiwindup_clamps_integral(self):
        ilim = 5
        f = IIR_PI_Filter(kp=10.0, ki=10.0, ilim=ilim, antiwindup=True)
        f.set_point = 10.0
        initial_integral = f.integral
        for _ in range(5):
            f.update(0.0)
        self.assertLessEqual(abs(f.control), ilim)
        self.assertAlmostEqual(f.integral, initial_integral, places=3)

    def test_no_antiwindup_allows_windup(self):
        '''
        No Anti-windup = Integral Continues Growing
        '''
        ilim = 5
        f = IIR_PI_Filter(kp=10.0, ki=10.0, ilim=ilim, antiwindup=False)
        f.set_point = 10.0
        for _ in range(5):
            f.update(0.0)
        self.assertGreater(abs(f.integral), 0)

    def test_trapezoid_integral(self):
        self.assertEqual(trapezoid_integral(2, 4, h=1), 3.0)

if __name__ == "__main__":
    unittest.main()
