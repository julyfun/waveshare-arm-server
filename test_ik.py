from fast_math.kin.scipy_ik_solver import PinoIkSolver
import pinocchio
import numpy as np

solver = PinoIkSolver('../roarm_main/roarm_description/urdf/roarm_description.urdf')

"""
['universe', 'base_link_to_link1', 'link1_to_link2', 'link2_to_link3', 'link3_to_gripper_link']
"""

"""
'universe', pinocchio.pinocchio_pywrap.FrameType.FIXED_JOINT)
('root_joint', pinocchio.pinocchio_pywrap.FrameType.FIXED_JOINT)
('base_link', pinocchio.pinocchio_pywrap.FrameType.BODY)
('base_link_to_link1', pinocchio.pinocchio_pywrap.FrameType.JOINT)
('link1', pinocchio.pinocchio_pywrap.FrameType.BODY)
('link1_to_link2', pinocchio.pinocchio_pywrap.FrameType.JOINT)
('link2', pinocchio.pinocchio_pywrap.FrameType.BODY)
('link2_to_link3', pinocchio.pinocchio_pywrap.FrameType.JOINT)
('link3', pinocchio.pinocchio_pywrap.FrameType.BODY)
('link3_to_gripper_link', pinocchio.pinocchio_pywrap.FrameType.JOINT)
('gripper_link', pinocchio.pinocchio_pywrap.FrameType.BODY)
('link3_to_hand_tcp', pinocchio.pinocchio_pywrap.FrameType.FIXED_JOINT)
('hand_tcp', pinocchio.pinocchio_pywrap.FrameType.BODY)
"""

q = solver.solve_ik(
    move_joints=['base_link_to_link1', 'link1_to_link2', 'link2_to_link3'],
    loss=[
        ("frame_target",
            ('base_link', pinocchio.pinocchio_pywrap.FrameType.BODY),
            ('hand_tcp', pinocchio.pinocchio_pywrap.FrameType.BODY),
            (np.array([0.2, 0.2, 0.2]), np.array([0.0, 0.0, 0.0, 1.0])),
            1.0, 0,
        ),
    ],
    minimize_options={
        "maxiter": 10,
    },
    verbose=True,
)
print(q)
print(solver.get_tcp(q, ('base_link', pinocchio.pinocchio_pywrap.FrameType.BODY), ('hand_tcp', pinocchio.pinocchio_pywrap.FrameType.BODY)))
