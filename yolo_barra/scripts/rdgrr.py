import numpy as np

r = np.array([68,82,94,105,117,129,139,145,153])

distance = np.array([20,30,40,50,60,70,80,90,100])

coef = np.polyfit(r, distance, 2)

print(coef)