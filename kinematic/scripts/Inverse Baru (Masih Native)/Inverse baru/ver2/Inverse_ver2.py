import numpy as np

class kinematika_robot:
    def __init__(self):
        self.r = 0.052
        self.L = 0.24
        self.pwm_max = 255
        self.cossin45 = 0.707106781186547524

    def perumusan(self, vx, vy, vtheta):
        matriks_sincos = np.array([
            [self.cossin45, self.cossin45, self.L],
            [-self.cossin45, self.cossin45, self.L],
            [-self.cossin45, -self.cossin45, self.L],
            [self.cossin45, -self.cossin45, self.L]
        ]) / self.r

        vx = np.clip(vx, -self.pwm_max, self.pwm_max)
        vy = np.clip(vy, -self.pwm_max, self.pwm_max) * (-1)
        vtheta = np.clip(vtheta, -self.pwm_max, self.pwm_max)

        matriks_input = np.array([vx, vy, vtheta])

        kecepatan_sudut_roda = matriks_sincos @ matriks_input
        kecepatan_sudut_roda = np.clip(kecepatan_sudut_roda, -self.pwm_max, self.pwm_max)
        

        w1 = kecepatan_sudut_roda[0]
        w2 = kecepatan_sudut_roda[1]
        w3 = kecepatan_sudut_roda[2]
        w4 = kecepatan_sudut_roda[3]

        return w1, w2, w3, w4