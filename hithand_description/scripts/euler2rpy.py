'''
This script calculates the rpy xyz values for urdf file of robot model,
with input of different conventions of DH parameters and euler matrix.
'''

import transforms3d
import numpy as np
import sympy as sp
from sympy.physics.mechanics import dynamicsymbols

def euler2rpy(mat):
    '''
    transform euler matrix to rpy angles
    :param mat: [3,3] matrix or [4,4] matrix
    :return: roll,pitch,yaw
    '''
    ai,aj,ak = transforms3d.euler.mat2euler(mat)
    print('roll,pitch,yaw are %f,%f,%f'%(ai,aj,ak))
    return ai,aj,ak


class DH2Traf(object):
    '''
    symbolic transformation from DH parameter to transformation matrix
    '''
    def __init__(self):
        self.theta, self.alpha, self.a, self.d = dynamicsymbols('theta, alpha, a, d')
        rot = sp.Matrix([[sp.cos(self.theta), -sp.sin(self.theta)*sp.cos(self.alpha),
                          sp.sin(self.theta)*sp.sin(self.alpha)],
                        [sp.sin(self.theta), sp.cos(self.theta)*sp.cos(self.alpha),
                         -sp.cos(self.theta)*sp.sin(self.alpha)],
                        [0, sp.sin(self.alpha), sp.cos(self.alpha)]])

        trans = sp.Matrix([self.a*sp.cos(self.theta),self.a*sp.sin(self.theta),self.d])

        last_row = sp.Matrix([[0, 0, 0, 1]])
        
        self.classic_dh_param_matrix = sp.Matrix.vstack(sp.Matrix.hstack(rot, trans), last_row)
        print('classic dh transform matrix', self.classic_dh_param_matrix)

        rot2 = sp.Matrix([[sp.cos(self.theta), -sp.sin(self.theta), 0],
                        [sp.sin(self.theta)*sp.cos(self.alpha), sp.cos(self.theta)*sp.cos(self.alpha),
                         -sp.sin(self.alpha)],
                        [sp.sin(self.theta)*sp.sin(self.alpha), sp.cos(self.theta)*sp.sin(self.alpha), sp.cos(self.alpha)]])
        trans2 = sp.Matrix([self.a, -self.d*sp.sin(self.alpha), self.d*sp.cos(self.alpha)])

        self.modified_dh_param_matrix = sp.Matrix.vstack(sp.Matrix.hstack(rot2, trans2), last_row)
        print('modified dh transform matrix', self.modified_dh_param_matrix)

    # def transf_one_joint(self,theta,alpha,a,d):
    #     transf = self.classic_dh_param_matrix.subs({alpha:self.alpha, a:a, theta:theta, d:d})

    def numericalize_classic_transf(self,theta,alpha,a,d):
        mat = sp.lambdify((self.theta,self.alpha,self.a,self.d), self.classic_dh_param_matrix, 'numpy')
        transf = mat(theta,alpha,a,d)
        print('classic method result transf',transf)
        euler2rpy(transf[:3,:3])

    def numericalize_modified_transf(self,theta,alpha,a,d):
        mat = sp.lambdify((self.theta, self.alpha, self.a, self.d), self.modified_dh_param_matrix, 'numpy')
        transf = mat(theta, alpha, a, d)
        print('modified method result transf',transf)
        euler2rpy(transf[:3, :3])

if __name__ == '__main__':
    # testing
    ThumbBaseTF = np.array([
        [0.429052,-0.571046,-0.699872,0.06217593],
        [0.187171,0.814201,-0.549586,0.044372912],
        [0.883675,0.104806,0.456218,0.078734808],
        [0,0,0,1]
    ])

    ForeBaseTF = np.array([
        [0, -0.087156, 0.996195, -0.002529881],
        [0, -0.996195, -0.087156, 0.03680013],
        [1, 0, 0, 0.107783545],
        [0, 0, 0, 1]
    ])

    MiddleBaseTF = np.array([
        [0,0,1,-0.0037],
        [0,-1,0,0.01],
        [1,0,0,0.117783545],
        [0, 0, 0, 1]
    ])

    RingBaseTF = np.array([
        [0, 0.087156, 0.996195, 0.02529881],
        [0,-0.996195,0.087156,-0.01680013],
        [1,0,0,0.11258354],
        [0, 0, 0, 1]
    ])
    LittleBaseTF = np.array([
        [0,0.173648,0.984808,0.000971571],
        [0,-0.984808,0.173648,-0.0433963],
        [1,0,0,0.093583545],
        [0, 0, 0, 1]
    ])




    # r, p, y = euler2rpy(ThumbBaseTF[:3,:3])
    # r,p,y = 0.225810, -1.083656, 0.411355
    # x,y,z = 0.06217593, 0.044372912, 0.078734808

    # r, p, y = euler2rpy(ForeBaseTF[:3, :3])
    # r,p,y = 3.054326,-1.570796,0.000000
    # x,y,z = -0.002529881, 0.03680013, 0.107783545
    # r, p, y = euler2rpy(MiddleBaseTF[:3, :3])
    # r, p, y = -3.141593,-1.570796,0.000000
    # x,y,z = -0.0037, 0.01, 0.117783545

    # r, p, y = euler2rpy(RingBaseTF[:3, :3])
    # r, p, y = -3.054326,-1.570796,0.000000
    # x,y,z = 0, -0.01680013, 0.11258354

    r, p, y = euler2rpy(LittleBaseTF[:3, :3])
    # r,p,y = -2.967060,-1.570796,0.000000
    # x,y,z = 0.000971571, -0.0433963, 0.093583545
    #
    # dh2traf = DH2Traf()
    # print('joint 0 to 1 transformation')
    # dh2traf.numericalize_classic_transf(theta=0, alpha=np.pi / 2, a=0, d=0)
    # dh2traf.numericalize_modified_transf(theta=0, alpha=np.pi / 2, a=0, d=0)
    # print('joint 1 to 2 transformation')
    # dh2traf.numericalize_classic_transf(theta=0, alpha=0, a=55, d=0)
    # dh2traf.numericalize_modified_transf(theta=0, alpha=0, a=55, d=0)
    # print('joint 2 to 3 transformation')
    # dh2traf.numericalize_classic_transf(theta=-np.pi/2, alpha=0, a=25, d=0)
    # dh2traf.numericalize_modified_transf(theta=-np.pi / 2, alpha=0, a=25, d=0)