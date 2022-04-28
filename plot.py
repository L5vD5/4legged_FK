import numpy as np
import matplotlib.pyplot as plt
# from utils import *

fig, ax = plt.subplots(figsize=(12, 6))
plt.title('current')
real_data = np.load('./real_trajectory.npy').reshape(-1, 8)
real_data -= 2048
real_data /= 2048
real_data *= 180
# real_data = np.load('./qpos_dlpg_18.npy').reshape(-1, 8)

curr_data = np.load('./real_curr.npy').reshape(-1, 8).astype(np.uint16)
# input_data = np.load('qpos_dlpg/qpos_dlpg_17.npy')
curr_data = curr_data.view(np.int16)
# input_data = get_shorttraj(input_data)

# ax.plot(real_data)
ax.plot(curr_data[:,3])

plt.show()