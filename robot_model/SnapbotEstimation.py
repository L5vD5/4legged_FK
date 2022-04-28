from SnapbotLeg import SnapbotLeg
import numpy as np
from utils import LinkNode, find_mother
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches

class Estimation():
  def __init__(self, qs, rpy, b):
    self.nJoint = 4
    self.qs = qs
    self.snapbot = SnapbotLeg(nJoint=self.nJoint, rpy=rpy, b=b)

  def forward(self):
    nJoint = self.nJoint
    qs = self.qs
    traj=np.empty([0,3])
    pjoints=np.empty([0,nJoint,3])
    kt = self.snapbot

    for i in range(len(qs)):
        for j in range(kt.N-1):
            kt.links[j+1] = LinkNode(id=j+1, name='link'+str(j+1), children=[j+2], b=kt.b[j].T,  a=kt.rpy[j], q=qs[i][0])
        kt.links[kt.N] = LinkNode(id=kt.N, name='link'+str(kt.N), children=[0], b=kt.b[kt.N-1].T,  a=kt.rpy[j], q=qs[i][1])
        kt.tree = find_mother(kt.links, 1)
        kt.tree[1].p = kt.root_positions.T
        kt.tree[1].R = np.eye(3)

        kt.forward_kinematics(1)
        pjoint = np.empty([0, 3])
        for j in range(nJoint):
            pjoint = np.concatenate((pjoint, np.reshape(kt.tree[j+1].p, (-1, 3))), axis=0)
        # print(pjoint.shape, kt.tree[j+1].p.shape)
        pjoints = np.concatenate((pjoints, np.reshape(pjoint, (-1, nJoint, 3))), axis=0)
        traj = np.concatenate((traj, np.reshape(kt.tree[nJoint].p, (-1, 3))), axis=0)

    return traj, pjoints

if __name__ == '__main__':
  real_data = np.load('./real_trajectory.npy').reshape(-1, 8)
  real_data -= 2048
  real_data /= 2048
  real_data *= np.pi
  real_data = - real_data
  qs = real_data
  e = Estimation(qs=real_data[:,0:2], rpy=[[1, 0, 0],[0, 0, 1],[1, 0, 0],[0,	0,	1]], b=[[1, 1, 0],[1, 1, 0],[1, 1, 0],[1,	1,	0]])
  e2 = Estimation(qs=real_data[:,2:4], rpy=[[1, 0, 0],[0, 0, 1],[1, 0, 0],[0,	0,	1]], b=[[-1, -1, 0],[-1, -1, 0],[-1, -1, 0],[-1,	-1,	0]])
  e3 = Estimation(qs=real_data[:,2:4], rpy=[[1, 0, 0],[0, 0, 1],[1, 0, 0],[0,	0,	1]], b=[[-1, 1, 0],[-1, 1, 0],[-1, 1, 0],[-1,	1,	0]])
  e4 = Estimation(qs=real_data[:,2:4], rpy=[[1, 0, 0],[0, 0, 1],[1, 0, 0],[0,	0,	1]], b=[[1, -1, 0],[1, -1, 0],[1, -1, 0],[1,	-1,	0]])

  traj, pjoints = e.forward()
  traj2, pjoints2 = e2.forward()
  traj3, pjoints3 = e3.forward()
  traj4, pjoints4 = e4.forward()

  fig = plt.figure()
  ax = fig.add_subplot(projection='3d')
  nJoint = e.nJoint

  def update(frame, data, data2, line, line2, data3, data4, line3, line4, data5, data6, line5, line6, data7, data8, line7, line8):
      line.set_data(data[:2, :frame])
      line.set_3d_properties(data[2, :frame])
      line2.set_data(data2[frame, 0:nJoint, 0], data2[frame, 0:nJoint, 1])
      line2.set_3d_properties(data2[frame, 0:nJoint, 2])
      line3.set_data(data3[:2, :frame])
      line3.set_3d_properties(data3[2, :frame])
      line4.set_data(data4[frame, 0:nJoint, 0], data4[frame, 0:nJoint, 1])
      line4.set_3d_properties(data4[frame, 0:nJoint, 2])
      line5.set_data(data5[:2, :frame])
      line5.set_3d_properties(data5[2, :frame])
      line6.set_data(data6[frame, 0:nJoint, 0], data6[frame, 0:nJoint, 1])
      line6.set_3d_properties(data6[frame, 0:nJoint, 2])
      line7.set_data(data7[:2, :frame])
      line7.set_3d_properties(data7[2, :frame])
      line8.set_data(data8[frame, 0:nJoint, 0], data8[frame, 0:nJoint, 1])
      line8.set_3d_properties(data8[frame, 0:nJoint, 2])


      return None;#line
  
  data = traj.T
  data2 = pjoints
  line, = ax.plot(data[0, 0:1], data[1, 0:1], data[2, 0:1], 'bo')
  line2, = ax.plot(data2[0, 0:nJoint, 0], data2[0, 0:nJoint, 1], data2[0, 0:nJoint, 2], '.-') # dot

  data3 = traj2.T
  data4 = pjoints2
  line3, = ax.plot(data3[0, 0:1], data3[1, 0:1], data3[2, 0:1], 'bo')
  line4, = ax.plot(data4[0, 0:nJoint, 0], data4[0, 0:nJoint, 1], data4[0, 0:nJoint, 2], '.-') # dot

  data5 = traj3.T
  data6 = pjoints3
  line5, = ax.plot(data5[0, 0:1], data5[1, 0:1], data5[2, 0:1], 'bo')
  line6, = ax.plot(data6[0, 0:nJoint, 0], data6[0, 0:nJoint, 1], data6[0, 0:nJoint, 2], '.-') # dot

  data7 = traj4.T
  data8 = pjoints4
  line7, = ax.plot(data7[0, 0:1], data7[1, 0:1], data7[2, 0:1], 'bo')
  line8, = ax.plot(data8[0, 0:nJoint, 0], data8[0, 0:nJoint, 1], data8[0, 0:nJoint, 2], '.-') # dot
  
  # v= np.array([
  #   data2[0, nJoint-1],
  #   data4[0, nJoint-1],
  #   data6[0, nJoint-1],
  #   data8[0, nJoint-1]
  # ])

  # patch = patches.Polygon(v,closed=True, fc='r', ec='r')
  # ax.add_patch(patch)

  ax.set_xlim3d([-1e1, 1e1])
  ax.set_ylim3d([-1e1, 1e1])
  ax.set_zlim3d([-1e1, 1e1])

  # plt.plot(eval_train_position[:,0], eval_train_position[:,1], eval_train_position[:,2], c='k')
  ani = FuncAnimation(fig, update, fargs=[data, data2, line, line2, data3, data4, line3, line4, data5, data6, line5, line6, data7, data8, line7, line8], frames=range(len(traj)), interval=1)

  plt.show()