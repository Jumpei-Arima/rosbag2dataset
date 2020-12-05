import numpy as np
import cv2
from tqdm import tqdm
from geometry_msgs.msg import Vector3
import tf
from cv_bridge import CvBridge, CvBridgeError

def convert_Image(data, height=None, width=None):
    obs = []
    bridge = CvBridge()
    for msg in tqdm(data):
        try:
            img = bridge.imgmsg_to_cv2(msg,"rgb8")
        except CvBridgeError as e:
            print(e)
        if height is not None and width is not None:
            h,w,c = img.shape
            img = img[0:h, int((w-h)*0.5):w-int((w-h)*0.5), :]
            img = cv2.resize(img, (height, width))
        obs.append(img)
    return obs

def convert_CompressedImage(data, height=None, width=None):
    obs = []
    for msg in tqdm(data):
        img = cv2.imdecode(np.fromstring(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if height is not None and width is not None:
            h,w,c = img.shape
            img = img[0:h, int((w-h)*0.5):w-int((w-h)*0.5), :]
            img = cv2.resize(img, (height, width))
        obs.append(img)
    return obs

def convert_Odometry(data, action_noise, lower_bound, upper_bound):
    acs = []
    pos = []
    goals = []
    for msg in tqdm(data):
        # action
        vel = np.array([msg.twist.twist.linear.x, msg.twist.twist.angular.z])
        vel = add_random_noise(vel, action_noise, lower_bound, upper_bound)
        acs.append(vel)
        # pose
        pose = get_pose_from_odom(msg)
        pos.append(pose)
    return acs, pos

def convert_Twist(data, action_noise, lower_bound, upper_bound):
    acs = []
    for msg in tqdm(data):
        # action
        vel = np.array([msg.linear.x, msg.angular.z])
        vel = add_random_noise(vel, action_noise, lower_bound, upper_bound)
        acs.append(vel)
    return acs

def convert_LaserScan(data):
    lidar = []
    for msg in tqdm(data):
        lidar.append(np.array(msg.ranges))
    return lidar

def convert_Imu(data):
    imu = []
    for msg in tqdm(data):
        # imu
        imu_data = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z])
        imu.append(imu_data)
    return imu

def transform_pose(pose, base_pose):
    x = pose[0] - base_pose[0]
    y = pose[1] - base_pose[1]
    yaw = pose[2] - base_pose[2]
    trans_pose = np.array([ x*np.cos(base_pose[2]) + y*np.sin(base_pose[2]),
                           -x*np.sin(base_pose[2]) + y*np.cos(base_pose[2]),
                           np.arctan2(np.sin(yaw), np.cos(yaw))])
    return trans_pose

def quaternion_to_euler(quaternion):
    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])

def angle_normalize(z):
    return np.arctan2(np.sin(z), np.cos(z))

def get_pose_from_odom(odom):
    yaw = quaternion_to_euler(odom.pose.pose.orientation).z
    pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])
    return pose

def add_random_noise(action, std, lb, ub):
    action += np.random.randn(*action.shape) * std
    return action.clip(lb, ub)