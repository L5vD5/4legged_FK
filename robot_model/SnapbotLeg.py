import numpy as np
from utils import rpy2rot, LinkNode, rodrigues_torch, rodrigues, find_mother

class SnapbotLeg():
    """Custom Model Class"""

    def __init__(self, nJoint, rpy, b, tree_name='leg'):
        """
        Args:
            N: the num of joint
            q: joint angles
            root_name (str, optional): name of root joint. Defaults to 'Hips'.
            device (str, optional): device to use. Defaults to 'cpu'.
        """
        self.root_name = tree_name

        self.rpy = np.array(rpy)# 2 motors' rpy
        self.b = np.array(b)# 2 link length
        self.N = nJoint
        print(rpy, b)

        # root position
        self.root_positions = np.array([0, 0, 0])

        self.links = {}
        self.links[0] = LinkNode(id=0,  name='NULL')

    def forward_kinematics(self, node_id):
        if node_id == 0: # For end of the kinematic chain. (NULL)
            return None
        if node_id != 1: # If node is not body
            mother_id = self.tree[node_id].mother
            self.tree[node_id].p = self.tree[mother_id].R @ self.tree[node_id].b + self.tree[mother_id].p
            self.tree[node_id].R = self.tree[mother_id].R @ rodrigues(self.tree[node_id].a, self.tree[node_id].q)
            # print(self.tree[node_id].R)

        for child_id in self.tree[node_id].children:
            self.forward_kinematics(child_id)

if __name__ == '__main__':
    leg = SnapbotLeg()
    # print(leg.rpy)