import math as mt

nilaiTrack = input('Nilai Track :')
track = float(nilaiTrack) * 0.0254
via = 2*mt.pi*track
viaIn_mill = via / 0.0254
print("Via in mm : %4f" %(via))
print("Via in mill : %4f" %(viaIn_mill))