from scipy import signal
import numpy as np


a1 = [1,2,3,4]
a2 = [2,3,4,1]

print(np.correlate(a1, a2, "same"))