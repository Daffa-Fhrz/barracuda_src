class baca_encoder:
    def __init__(self):
        self.cossin45 = 0.707106781186547524

    def perumusan(self, r1, r2):
        xencoder = self.cossin45 * r1 + self.cossin45 * r2
        yencoder = self.cossin45 * r1 - self.cossin45 * r2
        return xencoder, yencoder