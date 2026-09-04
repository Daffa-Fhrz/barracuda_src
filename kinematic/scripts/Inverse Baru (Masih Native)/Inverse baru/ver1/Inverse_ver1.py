import numpy as np

class kinematika_robot:
    def __init__(self):
        self.r = 5.2            #Radius dalam cm       
        self.L = 24             #Jarak pusat Robot - Roda cm
        self.pwm_max = 255
        self.xy_max = 1870
        self.theta_max = 55
        self.cossin45 = 0.70710678118654752

    def perumusan(self, vx, vy, vtheta):
        pos_array = [vx, vy]
        long_pos = abs(pos_array[0])

        for pos in pos_array:
            if abs(pos) > long_pos:
                long_pos = abs(pos)
        if long_pos > self.xy_max:
            vx = vx/long_pos * self.xy_max
            vy = (vy/long_pos * self.xy_max)* (-1)
            
        vtheta = np.clip(vtheta, -self.pwm_max, self.theta_max)

        w1 = (vx * self.cossin45 + vy * self.cossin45 + vtheta * self.L) / self.r
        w2 = (-vx * self.cossin45 + vy * self.cossin45 + vtheta * self.L) / self.r
        w3 = (-vx * self.cossin45 - vy * self.cossin45 + vtheta * self.L) / self.r
        w4 = (vx * self.cossin45 - vy * self.cossin45 + vtheta * self.L) / self.r

        speed_arrray = [w1, w2, w3, w4]

        hgh_speed = abs(speed_arrray[0])    

        for speed in speed_arrray:
            if abs(speed) > hgh_speed:
                hgh_speed = abs(speed)
        
        if hgh_speed > self.pwm_max:
            w1 = w1/hgh_speed *  self.pwm_max
            w2 = w2/hgh_speed *  self.pwm_max
            w3 = w3/hgh_speed *  self.pwm_max
            w4 = w4/hgh_speed *  self.pwm_max
        else:
            pass

        return w1, w2, w3, w4