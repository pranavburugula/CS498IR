import numpy as np

data = np.genfromtxt("stat_for_calc.csv", delimiter=',')
print(np.mean(data, 0))
print(np.std(data, 0))