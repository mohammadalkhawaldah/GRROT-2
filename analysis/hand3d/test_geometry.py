"""Analytic checks of coordinate conventions and bend geometry."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from geometry import palm_pose, bend, angular_distance


class GeometryTests(unittest.TestCase):
    def test_rigid_transform_equivariance(self):
        p=np.zeros((21,3));p[5]=[.03,.06,0];p[9]=[0,.08,0];p[13]=[-.015,.065,0];p[17]=[-.03,.05,0]
        center,r,q,_=palm_pose(p)
        rotation=Rotation.from_euler('xyz',[.4,-.7,1.1]).as_matrix();translation=np.array([.1,.2,.3])
        moved=p@rotation.T+translation
        c2,r2,q2,_=palm_pose(moved)
        np.testing.assert_allclose(c2,rotation@center+translation,atol=1e-12)
        np.testing.assert_allclose(r2,rotation@r,atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(r2),1)
        self.assertAlmostEqual(angular_distance(q2,-q2),0)

    def test_finger_bend(self):
        self.assertAlmostEqual(bend(np.array([-1.,0,0]),np.zeros(3),np.array([1.,0,0])),0)
        self.assertAlmostEqual(bend(np.array([-1.,0,0]),np.zeros(3),np.array([0.,1,0])),90)

    def test_degenerate_palm_rejected(self):
        with self.assertRaises(ValueError):palm_pose(np.zeros((21,3)))


if __name__=='__main__':unittest.main()
