import numpy as np

class Eksternal:
    def __init__(self):
        self.cossin45 = 0.707106781186547524
        self.ppr = 120
        self.R = 0.03

    def perumusan(self, enc1, enc2):
        r1 = (enc1 / self.ppr) * (2 * np.pi * self.R)
        r2 = (enc2 / self.ppr) * (2 * np.pi * self.R)
        xencoder = (self.cossin45 * r1 - self.cossin45 * r2) * (1/2)
        yencoder = (self.cossin45 * r1 + self.cossin45 * r2) * (1/2)
        return xencoder, yencoder
    
class Internal:
    def __init__(self):
        self.cossin45 = 0.707106781186547524
        self.ppr = 265
        self.R = 0.052

    def perumusan(self, enc1, enc2, enc3, enc4):
        r1 = (enc1 / self.ppr) * (2 * np.pi * self.R)
        r2 = (enc2 / self.ppr) * (2 * np.pi * self.R)
        r3 = (enc3 / self.ppr) * (2 * np.pi * self.R)
        r4 = (enc4 / self.ppr) * (2 * np.pi * self.R)
        xencoder = (self.cossin45 * (r2 - r4) + self.cossin45 * (r3 - r1)) * (1/2)
        yencoder = (self.cossin45 * (r2 - r4) - self.cossin45 * (r3 - r1)) * (1/2)
        return xencoder, yencoder
