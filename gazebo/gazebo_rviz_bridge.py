#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import re
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Point
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy

# Define constants from geckogrip_climb_animator.cc
kMoveStart = 0.24
kMoveEnd = 0.90
kLiftY = 0.025
kLiftZ = 0.070
kStepDuration = 3.10
kPhaseCount = 4
kGaitOrder = [0, 3, 1, 2] # FL, RR, FR, RL dog-like four-beat climb
kBodySide = 0.065
kHipSide = 0.075
kKneeSideMoving = 0.012
kKneeSideRest = 0.008
kKneeDropMoving = 0.055
kKneeDropRest = 0.038
kKneeLiftMoving = 0.040
kKneeLiftRest = 0.012
kFootYaw = 0.025
kYawScale = 0.04
kYawLimit = 0.08
kBodyLerp = 0.20
kGravityBodySag = 0.045
kSwingBodySag = 0.028
kGripPeelY = 0.014
kFallSteps = [4, 8, 22]
kFinalFallStep = 22
kFallDuration = 2.6
kWakePauseDuration = 1.2
kRecoverDuration = 4.4
kFloorBodyY = -1.55
kFloorBodyZ = 0.34
kRobotMass = 12.0
kGravityAccelBase = 9.81
kGravityAccelMax = 16.0
kFallClimbTime = kFinalFallStep * kStepDuration
kDemoCycleDuration = kFallClimbTime + len(kFallSteps) * (kFallDuration + kWakePauseDuration + kRecoverDuration)
kAdhesiveShearPerFoot = 42.0

# Holds coordinates
HOLDS = [
    (-0.72, 0.45), (-0.36, 0.45), (0.00, 0.45), (0.36, 0.45), (0.72, 0.45),
    (-0.58, 0.86), (-0.18, 0.86), (0.22, 0.86), (0.62, 0.86),
    (-0.72, 1.27), (-0.36, 1.27), (0.00, 1.27), (0.36, 1.27), (0.72, 1.27),
    (-0.58, 1.68), (-0.18, 1.68), (0.22, 1.68), (0.62, 1.68),
    (-0.72, 2.09), (-0.36, 2.09), (0.00, 2.09), (0.36, 2.09), (0.72, 2.09),
    (-0.58, 2.50), (-0.18, 2.50), (0.22, 2.50), (0.62, 2.50),
    (-0.72, 2.91), (-0.36, 2.91), (0.00, 2.91), (0.36, 2.91), (0.72, 2.91),
    (-0.58, 3.32), (-0.18, 3.32), (0.22, 3.32), (0.62, 3.32),
    (-0.72, 3.73), (-0.36, 3.73), (0.00, 3.73), (0.36, 3.73), (0.72, 3.73),
    (-0.58, 4.14), (-0.18, 4.14), (0.22, 4.14), (0.62, 4.14),
    (-0.72, 4.55), (-0.36, 4.55), (0.00, 4.55), (0.36, 4.55), (0.72, 4.55),
    (-0.58, 4.96), (-0.18, 4.96), (0.22, 4.96), (0.62, 4.96),
    (-0.72, 5.37), (-0.36, 5.37), (0.00, 5.37), (0.36, 5.37), (0.72, 5.37),
    (-0.58, 5.78), (-0.18, 5.78), (0.22, 5.78), (0.62, 5.78),
    (-0.72, 6.19), (-0.36, 6.19), (0.00, 6.19), (0.36, 6.19), (0.72, 6.19),
    (-0.58, 6.60), (-0.18, 6.60), (0.22, 6.60), (0.62, 6.60),
    (-0.72, 7.01), (-0.36, 7.01), (0.00, 7.01), (0.36, 7.01), (0.72, 7.01),
    (-0.58, 7.42), (-0.18, 7.42), (0.22, 7.42), (0.62, 7.42),
    (-0.72, 7.83), (-0.36, 7.83), (0.00, 7.83), (0.36, 7.83), (0.72, 7.83),
    (-0.58, 8.24), (-0.18, 8.24), (0.22, 8.24), (0.62, 8.24),
    (-0.72, 8.65), (-0.36, 8.65), (0.00, 8.65), (0.36, 8.65), (0.72, 8.65),
]

# Limb route mappings
ROUTES = {
    "FL": [15, 24, 33, 42, 51, 60, 69, 78, 87],
    "FR": [16, 25, 34, 43, 52, 61, 70, 79, 88],
    "RL": [6, 15, 24, 33, 42, 51, 60, 69, 78],
    "RR": [7, 16, 25, 34, 43, 52, 61, 70, 79],
}

LIMB_SPECS = [
    {"key": "FL", "side": -1.0, "shoulderZ": 0.32},
    {"key": "FR", "side": 1.0, "shoulderZ": 0.32},
    {"key": "RL", "side": -1.0, "shoulderZ": -0.32},
    {"key": "RR", "side": 1.0, "shoulderZ": -0.32},
]

def hold_point(index):
    x, z = HOLDS[index]
    return [x, -0.18, z]

def narrow_wall_foot(limb_spec, foot):
    return foot

def floor_foot_for(limb_spec, body_x=0.0, time=0.0, moving=False):
    diagonal = limb_spec["key"] in ("FL", "RR")
    phase = 0.0 if diagonal else math.pi
    cycle = math.sin(time * 4.2 + phase) if moving else 0.0
    lift = max(0.0, cycle) * 0.055
    stride = math.sin(time * 4.2 + phase) * 0.10 if moving else 0.0
    return [
        body_x + limb_spec["side"] * 0.16,
        kFloorBodyY + limb_spec["shoulderZ"] * 0.42 + stride,
        0.06 + lift,
    ]

def smoothstep(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)

def lerp(a, b, t):
    return a + (b - a) * t

def gravity_at_time(elapsed):
    if kFallClimbTime <= 0.0:
        return kGravityAccelBase
    t = max(0.0, min(1.0, elapsed / kFallClimbTime))
    return kGravityAccelBase + (kGravityAccelMax - kGravityAccelBase) * t

def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw

def inverse_leg_kinematics(x, y, z):
    l1 = 0.213
    l2 = 0.213
    
    hip = math.atan2(y, -z)
    z_rot = z * math.cos(hip) - y * math.sin(hip)
    
    sagittal = math.sqrt(x*x + z_rot*z_rot)
    c_knee = (sagittal*sagittal - l1*l1 - l2*l2) / (2.0 * l1 * l2)
    c_knee = max(-0.98, min(0.98, c_knee))
    knee = -math.acos(c_knee)
    
    thigh = math.atan2(-z_rot, x) - math.atan2(l2 * math.sin(-knee), l1 + l2 * math.cos(-knee))
    return hip, thigh, knee

class GazeboRvizBridge(Node):
    def __init__(self):
        super().__init__('gazebo_rviz_bridge')
        
        self.joint_pub = self.create_publisher(JointState, '/robot0/joint_states', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'stability_markers', 10)
        self.gravity_pub = self.create_publisher(String, 'gravity_value', 10)
        robot_desc_qos = QoSProfile(depth=1)
        robot_desc_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.robot_desc_pub = self.create_publisher(String, '/robot0/robot_description', robot_desc_qos)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        self.smoothBodyX = 0.0
        self.smoothBodyZ = 0.0
        self.smoothYaw = 0.0
        self.poseInitialized = False
        self.robot_description = self.load_robot_description()
        
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.description_timer = self.create_timer(1.0, self.publish_robot_description)
        self.publish_robot_description()
        self.get_logger().info("Gazebo-RViz bridge node started successfully.")

    def load_robot_description(self):
        urdf_path = '/mnt/c/Users/prath/Desktop/geckogrip-climber/external/go2_description/urdf/go2_description.urdf'
        try:
            with open(urdf_path, 'r') as f:
                urdf = f.read().replace(
                    'package://go2_description',
                    'file:///mnt/c/Users/prath/Desktop/geckogrip-climber/external/go2_description')
                return re.sub(
                    r'(<link\s+name="base_link">\s*)<inertial>.*?</inertial>',
                    r'\1',
                    urdf,
                    count=1,
                    flags=re.DOTALL)
        except OSError as exc:
            self.get_logger().error(f"Could not read robot URDF for RViz: {exc}")
            return ''

    def publish_robot_description(self):
        if not self.robot_description:
            return
        msg = String()
        msg.data = self.robot_description
        self.robot_desc_pub.publish(msg)

    def completed_steps(self, order, step):
        if step <= 0:
            return 0
        per_cycle = kGaitOrder.count(order)
        if per_cycle == 0:
            return 0
        full = step // kPhaseCount
        rem = step % kPhaseCount
        count = full * per_cycle
        for i in range(rem):
            if kGaitOrder[i] == order:
                count += 1
        return count

    def max_steps(self):
        max_cycles = 0
        for spec in LIMB_SPECS:
            order = 0 if spec["key"] == "FL" else (1 if spec["key"] == "FR" else (2 if spec["key"] == "RL" else 3))
            moves = max(0, len(ROUTES[spec["key"]]) - 1)
            per_cycle = kGaitOrder.count(order)
            if per_cycle > 0:
                max_cycles = max(max_cycles, math.ceil(moves / per_cycle))
        return max_cycles * kPhaseCount

    def foot_for(self, limb_spec, step, active_order, progress):
        key = limb_spec["key"]
        order = 0 if key == "FL" else (1 if key == "FR" else (2 if key == "RL" else 3))
        completed = self.completed_steps(order, step)
        route = ROUTES[key]
        from_idx = min(completed, len(route) - 1)
        active = (order == active_order) and (from_idx + 1 < len(route))
        
        a = narrow_wall_foot(limb_spec, hold_point(route[from_idx]))
        if not active:
            return a
            
        b = narrow_wall_foot(limb_spec, hold_point(route[from_idx + 1]))
        if progress <= kMoveStart:
            return a
        if progress >= kMoveEnd:
            return b
            
        t = smoothstep((progress - kMoveStart) / (kMoveEnd - kMoveStart))
        foot_x = lerp(a[0], b[0], t)
        foot_y = lerp(a[1], b[1], t) - kLiftY * math.sin(t * math.pi)
        foot_z = lerp(a[2], b[2], t) + kLiftZ * math.sin(t * math.pi)
        return [foot_x, foot_y, foot_z]

    def timer_callback(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        elapsed = max(0.0, now_sec - self.start_time)
        cycle_elapsed = elapsed % kDemoCycleDuration if kDemoCycleDuration > 0.0 else elapsed
        falling = False
        fallen = False
        recovering = False
        phase_time = 0.0
        climb_elapsed = cycle_elapsed
        remaining = cycle_elapsed
        for fall_step in kFallSteps:
            segment_climb_time = fall_step * kStepDuration
            fall_climb_time = fall_step * kStepDuration
            if remaining < segment_climb_time:
                climb_elapsed = remaining
                break
            remaining -= segment_climb_time

            if remaining < kFallDuration:
                falling = True
                phase_time = remaining
                climb_elapsed = fall_climb_time
                break
            remaining -= kFallDuration

            if remaining < kWakePauseDuration:
                fallen = True
                phase_time = kFallDuration
                climb_elapsed = fall_climb_time
                break
            remaining -= kWakePauseDuration

            if remaining < kRecoverDuration:
                recovering = True
                phase_time = remaining
                climb_elapsed = 0.0
                break
            remaining -= kRecoverDuration
            climb_elapsed = 0.0
        gravity_accel = gravity_at_time(climb_elapsed)
        
        step_index = int(math.floor(climb_elapsed / kStepDuration))
        max_steps = self.max_steps()
        step = min(step_index, max_steps)
        
        progress = 1.0 if step_index >= max_steps else (climb_elapsed % kStepDuration) / kStepDuration
        active_order = kGaitOrder[step % kPhaseCount]
        swinging = step < max_steps and progress > kMoveStart and progress < kMoveEnd
        swing_t = smoothstep((progress - kMoveStart) / (kMoveEnd - kMoveStart)) if swinging else 0.0
        phase_name = "waking up" if recovering else ("fallen" if fallen else ("falling" if falling else ("swing" if swinging else "four-paw grip")))
        
        # Calculate foot positions
        feet = []
        for spec in LIMB_SPECS:
            feet.append(self.foot_for(spec, step, active_order, progress))

        body_y = -0.34
        body_roll = math.pi / 2
        body_pitch = -math.pi / 2
        if falling or fallen or recovering:
            fall_t = smoothstep(phase_time / kFallDuration) if falling else 1.0
            recover_t = smoothstep(phase_time / kRecoverDuration) if recovering else 0.0
            foot_t = 1.0 - recover_t if recovering else fall_t
            body_y = (
                kFloorBodyY + (-0.34 - kFloorBodyY) * recover_t
                if recovering
                else -0.34 + (kFloorBodyY + 0.34) * fall_t
            )
            body_roll = lerp(0.0, math.pi / 2, recover_t) if recovering else lerp(math.pi / 2, 0.0, fall_t)
            body_pitch = lerp(0.0, -math.pi / 2, recover_t) if recovering else lerp(-math.pi / 2, 0.0, fall_t)
            for i, spec in enumerate(LIMB_SPECS):
                floor_foot = floor_foot_for(spec, time=phase_time, moving=recovering)
                feet[i] = [
                    lerp(feet[i][0], floor_foot[0], foot_t),
                    lerp(feet[i][1], floor_foot[1], foot_t),
                    lerp(feet[i][2], floor_foot[2], foot_t),
                ]
            
        # Target body positions
        targetBodyX = 0.0
        targetBodyZ = 0.0
        for i, spec in enumerate(LIMB_SPECS):
            targetBodyX += feet[i][0] - spec["side"] * kBodySide
            targetBodyZ += feet[i][2] - spec["shoulderZ"]
        gravity_scale = gravity_accel / kGravityAccelBase if kGravityAccelBase > 0.0 else 1.0
        gravity_body_sag = kGravityBodySag * gravity_scale
        gravity_swing_sag = kSwingBodySag * gravity_scale
        targetBodyX /= 4.0
        targetBodyZ = (targetBodyZ / 4.0) + 0.14
        targetBodyZ -= gravity_body_sag + (gravity_swing_sag * math.sin(swing_t * math.pi) if swinging else 0.0)
        body_y -= (kGripPeelY * math.sin(swing_t * math.pi)) if swinging else (kGripPeelY * 0.35)
        
        leftZ = (feet[0][2] + feet[2][2]) * 0.5
        rightZ = (feet[1][2] + feet[3][2]) * 0.5
        targetYaw = max(-kYawLimit, min(kYawLimit, (leftZ - rightZ) * kYawScale))
        
        if not self.poseInitialized:
            self.smoothBodyX = targetBodyX
            self.smoothBodyZ = targetBodyZ
            self.smoothYaw = targetYaw
            self.poseInitialized = True
        else:
            self.smoothBodyX += (targetBodyX - self.smoothBodyX) * kBodyLerp
            self.smoothBodyZ += (targetBodyZ - self.smoothBodyZ) * kBodyLerp;
            self.smoothYaw += (targetYaw - self.smoothYaw) * kBodyLerp;

        # 1. Publish TF odom -> base_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'robot0/base_link'
        t.transform.translation.x = self.smoothBodyX
        t.transform.translation.y = body_y
        t.transform.translation.z = self.smoothBodyZ
        
        qx, qy, qz, qw = euler_to_quaternion(body_roll, body_pitch, self.smoothYaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)
        
        # 2. Compute Leg Joint angles using IK
        joint_names = []
        joint_positions = []
        
        for i, spec in enumerate(LIMB_SPECS):
            key = spec["key"]
            side = spec["side"]
            shoulder_z = spec["shoulderZ"]
            
            # Hip world coords
            hip_world_x = self.smoothBodyX + side * kHipSide
            hip_world_y = body_y - 0.01
            hip_world_z = self.smoothBodyZ + shoulder_z
            
            # Relative to hip in world
            dx = feet[i][0] - hip_world_x
            dy = feet[i][1] - hip_world_y
            dz = feet[i][2] - hip_world_z
            
            # Map relative coords to local robot frame (where robot X is up-wall Z, robot Y is lateral X, robot Z is out-wall Y)
            # Since the robot body is rotated:
            # - Local X is along world Z
            # - Local Y is along world X (adjusted for left/right side)
            # - Local Z is along world -Y (towards the wall)
            local_x = dz
            local_y = dx if side > 0 else -dx
            local_z = dy  # Since world Y increases away from the wall, dy is negative when extending towards it
            
            hip_angle, thigh_angle, knee_angle = inverse_leg_kinematics(local_x, local_y, local_z)
            
            # Correct joints in URDF are named: key_hip_joint, key_thigh_joint, key_calf_joint
            joint_names.extend([f"{key}_hip_joint", f"{key}_thigh_joint", f"{key}_calf_joint"])
            joint_positions.extend([hip_angle, thigh_angle, knee_angle])
            
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = joint_names
        js.position = joint_positions
        self.joint_pub.publish(js)

        weight_force = kRobotMass * gravity_accel
        attached_feet = 0 if falling or fallen else (4 if recovering or not swinging else 3)
        load_per_foot = 0.0 if attached_feet == 0 else weight_force / attached_feet
        grip_load = 0.0 if attached_feet == 0 else min(1.0, weight_force / (attached_feet * kAdhesiveShearPerFoot))
        grip_text = "released/falling" if attached_feet == 0 else f"{grip_load * 100:.0f}%"
        climb_height = max(0.0, self.smoothBodyZ - HOLDS[0][1])
        sag = gravity_body_sag + (gravity_swing_sag * math.sin(swing_t * math.pi) if swinging else 0.0)

        gravity_msg = String()
        gravity_msg.data = (
            f"phase={phase_name}, gravity={gravity_accel:.2f} m/s^2, "
            f"F=mg={weight_force:.1f} N, contacts={attached_feet}, "
            f"load_per_foot={load_per_foot:.1f} N, grip_load={grip_text}, "
            f"height={climb_height:.2f} m, sag={sag:.3f} m"
        )
        self.gravity_pub.publish(gravity_msg)
        
        # 3. Publish Stability and Gravity Focus markers
        markers = MarkerArray()

        body_marker = Marker()
        body_marker.header.frame_id = "odom"
        body_marker.header.stamp = js.header.stamp
        body_marker.ns = "simple_robot"
        body_marker.id = 10
        body_marker.type = Marker.CUBE
        body_marker.action = Marker.ADD
        body_marker.pose.position.x = self.smoothBodyX
        body_marker.pose.position.y = body_y
        body_marker.pose.position.z = self.smoothBodyZ
        body_marker.pose.orientation.x = qx
        body_marker.pose.orientation.y = qy
        body_marker.pose.orientation.z = qz
        body_marker.pose.orientation.w = qw
        body_marker.scale.x = 0.58
        body_marker.scale.y = 0.22
        body_marker.scale.z = 0.30
        body_marker.color.r = 0.78
        body_marker.color.g = 0.84
        body_marker.color.b = 0.86
        body_marker.color.a = 0.95
        markers.markers.append(body_marker)

        leg_marker = Marker()
        leg_marker.header.frame_id = "odom"
        leg_marker.header.stamp = js.header.stamp
        leg_marker.ns = "simple_robot"
        leg_marker.id = 11
        leg_marker.type = Marker.LINE_LIST
        leg_marker.action = Marker.ADD
        leg_marker.scale.x = 0.035
        leg_marker.color.r = 0.03
        leg_marker.color.g = 0.05
        leg_marker.color.b = 0.08
        leg_marker.color.a = 1.0

        for i, spec in enumerate(LIMB_SPECS):
            hip = Point(
                x=self.smoothBodyX + spec["side"] * kHipSide,
                y=body_y - 0.01,
                z=self.smoothBodyZ + spec["shoulderZ"])
            foot = Point(x=feet[i][0], y=feet[i][1], z=feet[i][2])
            knee = Point(
                x=(hip.x + foot.x) * 0.5 + spec["side"] * (kKneeSideMoving if swinging else kKneeSideRest),
                y=(hip.y + foot.y) * 0.5 - (kKneeDropMoving if swinging else kKneeDropRest),
                z=(hip.z + foot.z) * 0.5 + (kKneeLiftMoving if swinging else kKneeLiftRest))
            leg_marker.points.extend([hip, knee, knee, foot])

            foot_marker = Marker()
            foot_marker.header.frame_id = "odom"
            foot_marker.header.stamp = js.header.stamp
            foot_marker.ns = "simple_robot"
            foot_marker.id = 20 + i
            foot_marker.type = Marker.SPHERE
            foot_marker.action = Marker.ADD
            foot_marker.pose.position.x = foot.x
            foot_marker.pose.position.y = foot.y
            foot_marker.pose.position.z = foot.z
            foot_marker.pose.orientation.w = 1.0
            foot_marker.scale.x = 0.11
            foot_marker.scale.y = 0.11
            foot_marker.scale.z = 0.06
            foot_marker.color.r = 0.12
            foot_marker.color.g = 0.92
            foot_marker.color.b = 0.20
            foot_marker.color.a = 1.0
            markers.markers.append(foot_marker)

        markers.markers.append(leg_marker)
        
        # CoM Marker (Sphere)
        com_marker = Marker()
        com_marker.header.frame_id = "odom"
        com_marker.header.stamp = js.header.stamp
        com_marker.ns = "gravity_focus"
        com_marker.id = 0
        com_marker.type = Marker.SPHERE
        com_marker.action = Marker.ADD
        com_marker.pose.position.x = self.smoothBodyX
        com_marker.pose.position.y = body_y
        com_marker.pose.position.z = self.smoothBodyZ
        com_marker.pose.orientation.w = 1.0
        com_marker.scale.x = 0.12
        com_marker.scale.y = 0.12
        com_marker.scale.z = 0.12
        com_marker.color.r = 0.0
        com_marker.color.g = 0.8
        com_marker.color.b = 1.0
        com_marker.color.a = 1.0
        markers.markers.append(com_marker)
        
        # Gravity vector Arrow
        grav_marker = Marker()
        grav_marker.header.frame_id = "odom"
        grav_marker.header.stamp = js.header.stamp
        grav_marker.ns = "gravity_focus"
        grav_marker.id = 1
        grav_marker.type = Marker.ARROW
        grav_marker.action = Marker.ADD
        grav_len = 0.8 * gravity_scale
        grav_marker.points = [
            Point(x=self.smoothBodyX, y=body_y, z=self.smoothBodyZ),
            Point(x=self.smoothBodyX, y=body_y, z=self.smoothBodyZ - grav_len) # gravity points downwards in world coords
        ]
        grav_marker.scale.x = 0.035  # shaft diameter
        grav_marker.scale.y = 0.065  # head diameter
        grav_marker.scale.z = 0.08   # head length
        grav_marker.color.r = 1.0
        grav_marker.color.g = 0.1
        grav_marker.color.b = 0.1
        grav_marker.color.a = 1.0
        markers.markers.append(grav_marker)

        grav_text = Marker()
        grav_text.header.frame_id = "odom"
        grav_text.header.stamp = js.header.stamp
        grav_text.ns = "gravity_focus"
        grav_text.id = 3
        grav_text.type = Marker.TEXT_VIEW_FACING
        grav_text.action = Marker.ADD
        grav_text.pose.position.x = self.smoothBodyX + 0.34
        grav_text.pose.position.y = body_y
        grav_text.pose.position.z = self.smoothBodyZ - 0.90
        grav_text.pose.orientation.w = 1.0
        grav_text.scale.z = 0.13
        grav_text.color.r = 1.0
        grav_text.color.g = 0.1
        grav_text.color.b = 0.1
        grav_text.color.a = 1.0
        grav_text.text = (
            f"RAMPING GRAVITY\n"
            f"g = {gravity_accel:.2f} m/s^2\n"
            f"g target = {kGravityAccelMax:.2f} m/s^2\n"
            f"weight F=mg = {weight_force:.1f} N\n\n"
            f"CHANGING EFFECT ON ROBOT\n"
            f"phase = {phase_name}\n"
            f"contact feet = {attached_feet}\n"
            f"load per foot = {load_per_foot:.1f} N\n"
            f"grip load = {grip_text}\n"
            f"height = {climb_height:.2f} m\n"
            f"body sag = {sag:.3f} m"
        )
        markers.markers.append(grav_text)
        
        # Foot support polygon (Line Strip)
        poly_marker = Marker()
        poly_marker.header.frame_id = "odom"
        poly_marker.header.stamp = js.header.stamp
        poly_marker.ns = "stability"
        poly_marker.id = 2
        poly_marker.type = Marker.LINE_STRIP
        poly_marker.action = Marker.ADD
        poly_marker.scale.x = 0.02
        poly_marker.color.r = 0.1
        poly_marker.color.g = 1.0
        poly_marker.color.b = 0.1
        poly_marker.color.a = 0.8
        
        # Support polygon links (only include contact feet, or all 4 if double support)
        # Order: FL -> FR -> RR -> RL -> FL
        active_idx = -1
        if progress < 1.0:
            active_key = ["FL", "FR", "RL", "RR"][active_order]
            if active_key == "FL": active_idx = 0
            elif active_key == "FR": active_idx = 1
            elif active_key == "RL": active_idx = 2
            elif active_key == "RR": active_idx = 3
            
        contact_points = []
        for i in [0, 1, 3, 2, 0]: # FL -> FR -> RR -> RL -> FL
            if i != active_idx:
                p = Point()
                p.x = feet[i][0]
                p.y = feet[i][1]
                p.z = feet[i][2]
                contact_points.append(p)
                
        poly_marker.points = contact_points
        markers.markers.append(poly_marker)
        
        self.marker_pub.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    bridge = GazeboRvizBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
