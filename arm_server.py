from fastapi import FastAPI
from pydantic import BaseModel
import threading
import time
import uvicorn
from typing import List, Optional
import tf2_ros
import rclpy
import numpy as np
from rclpy.node import Node
from roarm_moveit.srv import MovePointCmd
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from rclpy.clock import Clock

from loguru import logger
from fast_math.itp_state.api import ItpRingApi
from fast_math.kin.scipy_ik_solver import PinoIkSolver
from fast_math.time_utils import precise_sleep
from fast_math.counter import FpsCounter
import hydra
from omegaconf import DictConfig
import pinocchio
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TransformRequest(BaseModel):
    time: float
    frame_id: str
    child_frame_id: str
    xyz: List[float]  # [x, y, z]
    xyzw: List[float]  # [x, y, z, w]

class TransformQuery(BaseModel):
    frame_id: str
    child_frame_id: str

class MoveArmRequest(BaseModel):
    x: float
    y: float
    z: float

class GripRequest(BaseModel):
    data: float

class WaypointRequest(BaseModel):
    q0: float
    q1: float
    q2: float


class RosNode(Node):
    def __init__(self, cfg):
        super().__init__('fastapi_ros_node')
        self.cfg = cfg
        self.move_client = self.create_client(MovePointCmd, '/move_point_cmd')
        self.grip_publisher = self.create_publisher(Float32, '/gripper_cmd', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # [get q]
        self.joints_latest = None
        self.joints_sub =  self.create_subscription(
            JointState,
            '/joint_states',
            self.joints_sub_cb,
            10)

        # [traj]
        self.traj_pub = self.create_publisher(JointTrajectory, '/hand_controller/joint_trajectory', 10)

        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /move_point_cmd service...')


    def joints_sub_cb(self, msg):
        self.joints_latest = msg

    def get_q(self):
        if self.joints_latest:
            joint_names = self.joints_latest.name
            joint_positions = self.joints_latest.position
            joint_states = {name: position for name, position in zip(joint_names, joint_positions)}
            return np.array([joint_states[name] for name in self.cfg.joints])
        else:
            self.get_logger().info('No joint state received yet.')
            return None

    def pub_q_traj(self, q: List):
        msg = JointTrajectory()
        msg.joint_names = self.cfg.joints[:3]
        assert(len(q) == len(msg.joint_names))

        point = JointTrajectoryPoint()
        point.positions = q
        time_from_start = self.cfg.traj_time_rate * (1. / self.cfg.fps)
        point.time_from_start.sec = int(time_from_start)
        point.time_from_start.nanosec = int((time_from_start - int(time_from_start)) * 1e9)

        msg.points.append(point)

        self.traj_pub.publish(msg)


class ArmServer():
    def __init__(self, cfg: DictConfig):
        self.cfg = cfg
        self.app = FastAPI()
        rclpy.init()
        self.ros_node = RosNode(cfg)
        def spin():
            rclpy.spin(self.ros_node)
        thread = threading.Thread(target=spin)
        thread.start()
        time.sleep(1)

        # [ik & itp]
        self.arm_itp = ItpRingApi(cfg.fps, cfg.buf_time, cfg.v_max[:3], cfg.a_max[:3], cfg.j_max[:3])
        self.arm_ik = PinoIkSolver(cfg.urdf)
        self.state_q = None
        while self.state_q is None:
            logger.info('state_q Waiting for joint state...')
            time.sleep(0.1)
            self.state_q = self.ros_node.get_q()

        # [app]
        self.setup_routes()

        # [debug]
        self.fps_counter = FpsCounter()
        threading.Thread(target=self.move_arm_loop).start()


    # [move]
    def move_arm_loop(self):
        last = time.time()
        while True:
            cur = time.time()
            till = max(cur, last + 1. / self.cfg.fps)
            precise_sleep(till - cur)
            last = till
            tar = self.arm_itp.ring.pop_front()
            if len(tar):
                self.fps_counter.count_and_check('move_arm')
                self.ros_node.pub_q_traj(tar.tolist())

    def setup_routes(self):
    # [func]
        @self.app.post("/move-v2")
        async def move_v2(req: MoveArmRequest):

            x, y, z = req.x, req.y, req.z
            # real
            t_sec = self.ros_node.get_clock().now().nanoseconds / 1e9
            self.state_q = self.arm_ik.solve_ik(
                move_joints=self.cfg.ik_joints,
                loss=[
                    ("frame_target",
                        ('base_link', pinocchio.pinocchio_pywrap.FrameType.BODY),
                        ('hand_tcp', pinocchio.pinocchio_pywrap.FrameType.BODY),
                        (np.array([x, y, z]), np.array([0.0, 0.0, 0.0, 1.0])),
                        1.0, 0,
                    ),
                ],
                # ref_q=np.zeros_like(self.state_q),
                ref_q=self.state_q,
                minimize_options={
                    "maxiter": 30,
                },
                verbose=True,
            )

            init_q = self.ros_node.get_q()[:3] if self.arm_itp.ring.get_valid_len() == 0 else None
            self.arm_itp.interpolate(t_sec, self.state_q[:3], init_q, np.zeros_like(self.state_q[:3]))

            tcp = self.arm_ik.get_tcp(self.state_q, ("base_link", pinocchio.pinocchio_pywrap.FrameType.BODY), ("hand_tcp", pinocchio.pinocchio_pywrap.FrameType.BODY))
            logger.info(f'expect: {x, y, z}')
            logger.info(f'solved: {tcp}')
            return { "status": "success" }


        @self.app.post("/move")
        async def move(req: MoveArmRequest):
            request = MovePointCmd.Request()
            request.x = req.x
            request.y = req.y
            request.z = req.z

            future = self.ros_node.move_client.call(request)
            return {"status": "success", "data": [req.x, req.y, req.z]}

        @self.app.post("/grip")
        async def grip(req: GripRequest):
            msg = Float32()
            msg.data = req.data
            self.ros_node.grip_publisher.publish(msg)
            return {"status": "success", "data": req.data}

        # [tf2]
        @self.app.post("/pub-tf2")
        async def pub_tf2(req: TransformRequest):
            transform = TransformStamped()
            transform.header.stamp = rclpy.time.Time(seconds=req.time).to_msg()
            transform.header.frame_id = req.frame_id
            transform.child_frame_id = req.child_frame_id
            transform.transform.translation.x = req.xyz[0]
            transform.transform.translation.y = req.xyz[1]
            transform.transform.translation.z = req.xyz[2]
            transform.transform.rotation.x = req.xyzw[0]
            transform.transform.rotation.y = req.xyzw[1]
            transform.transform.rotation.z = req.xyzw[2]
            transform.transform.rotation.w = req.xyzw[3]

            self.ros_node.tf_broadcaster.sendTransform(transform)

            return {"status": "success", "frame_id": req.frame_id, "child_frame_id": req.child_frame_id}

        @self.app.post("/get-tf2")
        async def get_tf2(query: TransformQuery):
            try:
                # Wait for the transform to be available
                frames_yaml = self.ros_node.tf_buffer.all_frames_as_yaml()

                transform = self.ros_node.tf_buffer.lookup_transform(
                    query.frame_id,
                    query.child_frame_id,
                    # zero time
                    rclpy.time.Time(seconds=0)
                )

                translation = transform.transform.translation
                rotation = transform.transform.rotation

                return {
                    "status": "success",
                    "xyz": [translation.x, translation.y, translation.z],
                    "xyzw": [rotation.x, rotation.y, rotation.z, rotation.w]
                }
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                return {"status": "error", "message": str(e)}


        @self.app.post("/get-q")
        async def get_q():
            return {"status": "success", "data": self.ros_node.get_q()[:3].tolist()}

        @self.app.post("/waypoint")
        async def waypoint(req: WaypointRequest):
            init_q = self.ros_node.get_q()[:3] if self.arm_itp.ring.get_valid_len() == 0 else None
            self.arm_itp.interpolate(self.ros_node.get_clock().now().nanoseconds / 1e9, np.array([req.q0, req.q1, req.q2]), init_q, np.zeros_like(init_q))
            return {"status": "success", "data": [req.q0, req.q1, req.q2]}


    # [run]
    def run(self):
        uvicorn.run(self.app, host="0.0.0.0", port=4060)

        import atexit
        atexit.register(rclpy.shutdown)


@hydra.main(version_base=None, config_path=".", config_name="config")
def main(cfg: DictConfig):
    arm_server = ArmServer(cfg.arm_server)
    arm_server.run()

# [main]
if __name__ == "__main__":
    main()
