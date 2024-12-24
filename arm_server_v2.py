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

from fast_math.itp_state.api import ItpRingApi
from fast_math.kin.scipy_ik_solver import PinoIkSolver
from fast_math.time_utils import precise_sleep
from fast_math.counter import FpsCounter
import hydra
import json
from loguru import logger
from omegaconf import DictConfig
import pinocchio
import serial
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robotoy.dev import find_device_by_serial
from robotoy.time_utils import sleep_timer, Once, PassiveTimer

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
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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

        # [debug]
        self.fps_counter = FpsCounter()
        self.once = Once()

        # [itp ring]
        self.arm_itp = ItpRingApi(cfg.fps, cfg.buf_time, cfg.v_max[:3], cfg.a_max[:3], cfg.j_max[:3])

        # [arm serial]
        #
        self.rw_lock = threading.Lock()

        port = find_device_by_serial(cfg.arm_serial)
        self.ser = serial.Serial(port, baudrate=115200, dsrdtr=None)
        self.ser.setRTS(True)
        self.ser.setDTR(True)

        self.cache_q = None
        threading.Thread(target=self.read_loop, daemon=True).start()
        threading.Thread(target=self.write_feedback_loop, daemon=True).start()
        threading.Thread(target=self.write_move_loop, daemon=True).start()

        while self.cache_q is None:
            logger.info('Waiting for arm serial...')
            time.sleep(0.1)


        # [ik & itp]
        self.arm_ik = PinoIkSolver(cfg.urdf)
        self.arm_ik_state_q = self.cache_q
        logger.info(f"Initial state q: {self.arm_ik_state_q}")

        # [app]
        self.setup_routes()

        # threading.Thread(target=self.move_arm_loop).start()

    def read_loop(self):
        logger.info("read loop started")
        def fn():
            if not self.ser.in_waiting:
                return
            try:
                with self.rw_lock:
                    data = self.ser.readline().decode('utf-8')
                data = json.loads(data)
                if data.get('T') == 1051:
                    self.cache_q = np.array(
                        [
                            -data.get('b'),
                            -data.get('s'),
                            data.get('e'),
                            data.get('t'),
                        ]
                    )
            except json.JSONDecodeError as e:
                ... # ok
            except Exception as e:
                logger.error(f"Error: {e}")
        sleep_timer(0.005, fn)

    def write_feedback_loop(self):
        def fn():
            with self.rw_lock:
                self.ser.write('{"T":105}'.encode() + b'\n')
                self.once.try_act(lambda: logger.info(f"write 105"), '105')
        sleep_timer(0.05, fn)

    def write_move_loop(self):
        def fn():
            tar = self.arm_itp.ring.pop_front()
            if len(tar):
                with self.rw_lock:
                    self.once.try_act(lambda: logger.info(f"write 101 move: {tar}"), '101')
                    dir = [-1, -1, 1]
                    for i, q in enumerate(tar):
                        logger.info(f'{{"T":101,"joint":{i + 1},"rad":{q * dir[i]},"spd":0,"acc":0}}')
                        self.ser.write(f'{{"T":101,"joint":{i + 1},"rad":{q * dir[i]},"spd":0,"acc":0}}'.encode() + b'\n')

        sleep_timer(1. / self.cfg.fps, fn)

    def setup_routes(self):
    # [func]
        @self.app.post("/move")
        async def move_v3(req: MoveArmRequest):

            x, y, z = req.x, req.y, req.z
            # real
            t_sec = time.time()
            self.arm_ik_state_q = self.arm_ik.solve_ik(
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
                ref_q=self.arm_ik_state_q,
                minimize_options={
                    "maxiter": 30,
                },
                verbose=True,
            )

            init_q = self.cache_q[:3] if self.arm_itp.ring.get_valid_len() == 0 else None
            self.arm_itp.interpolate(t_sec, self.arm_ik_state_q[:3], init_q, np.zeros_like(self.arm_ik_state_q[:3]))

            tcp = self.arm_ik.get_tcp(self.arm_ik_state_q, ("base_link", pinocchio.pinocchio_pywrap.FrameType.BODY), ("hand_tcp", pinocchio.pinocchio_pywrap.FrameType.BODY))
            logger.info(f'expect: {x, y, z}')
            logger.info(f'solved: {tcp}')
            logger.info(f'q: {self.arm_ik_state_q}')
            return { "status": "success" }


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

        @self.app.post("/get-gripper")
        async def get_gripper():
            return { "status": "success", "data": self.ros_node.get_q()[3] }

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
