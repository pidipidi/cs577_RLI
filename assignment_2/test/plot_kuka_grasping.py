from baselines.common import plot_util as pu
import matplotlib.pyplot as plt
import numpy as np


results = pu.load_results( "./logs/dqn")

r = results[0]
print ("----------------")
print (type(r.progress))
## print (np.array(r.progress['episodes']))
## print (r.progress['mean 100 episode reward'])

#plt.plot(np.cumsum(r.monitor.l), r.monitor.r)
plt.plot(list(r.progress['episodes']), list(r.progress['mean 100 episode reward']))
plt.show()
