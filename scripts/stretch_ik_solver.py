#!/usr/bin/env python3
import kdl_parser_py.urdf
import numpy as np
import PyKDL as kdl
import rospkg
import os.path


class IKSolver:
    def __init__(
        self,
        urdf_path=None,
        verbose=False
    ) -> None:
        rospack = rospkg.RosPack()
        # the urdf is generated in my macbook using the original jupyter notebook
        _urdf_path = (
            os.path.join(rospack.get_path("stretch_shared_autonomy"), "urdf", "stretch_pruned.urdf")
            if urdf_path is None
            else urdf_path
        )
        ok, tree = kdl_parser_py.urdf.treeFromFile(_urdf_path)
        self.__chain = tree.getChain("base_link", "link_grasp_center")
        self.__jac_solver = kdl.ChainJntToJacSolver(self.__chain)
        self.joint_num = self.__chain.getNrOfJoints()
        self.verbose = verbose

    def J_kdl_to_np(self, J, fixed_joints=()) -> np.ndarray:
        return np.array(
            [
                [J[i, j] if j not in fixed_joints else 0 for j in range(J.columns())]
                for i in range(J.rows())
            ]
        )

    def solve_J_pinv(self, q, fixed_joints=()) -> np.ndarray:
        if len(q) != self.joint_num:
            raise ValueError(f"input q dimension is {len(q)}, expecting {self.joint_num}")

        q_kdl = kdl.JntArray(self.joint_num)
        for i, j in enumerate(q):
            q_kdl[i] = j
        J_kdl = kdl.Jacobian(self.joint_num)
        self.__jac_solver.JntToJac(q, J_kdl)
        if self.verbose:
            print(J_kdl)
        J = self.J_kdl_to_np(J_kdl, fixed_joints=fixed_joints)
        J_pinv = np.linalg.pinv(J)
        if self.verbose:
            with np.printoptions(precision=4, suppress=True, linewidth=100):
                print(J)
                print(J_pinv)
        return J_pinv

if __name__ == "__main__":
    ik_solver = IKSolver(verbose=True)
    ik_solver.__jac_solver
