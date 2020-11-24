import numpy as np
import cv2
from tqdm import tqdm
from geometry_msgs.msg import Vector3
import tf

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

def convert2image(sample_data, height, width):
    obs = []
    for data in tqdm(sample_data):
        img = cv2.imdecode(np.fromstring(data.data, np.uint8), cv2.IMREAD_COLOR)
        h,w,c = img.shape
        img = img[0:h, int((w-h)*0.5):w-int((w-h)*0.5), :]
        img = cv2.resize(img, (height, width))
        obs.append(img)
    return obs

def convert_from_odom(sample_data, action_noise, lower_bound, upper_bound, goal_steps=None):
    acs = []
    pos = []
    goals = []
    for data in tqdm(sample_data):
        # action
        vel = np.array([data.twist.twist.linear.x, data.twist.twist.angular.z])
        vel = add_random_noise(vel, action_noise, lower_bound, upper_bound)
        acs.append(vel)
        # pose
        pose = get_pose_from_odom(data)
        pos.append(pose)
    if goal_steps is not None:
        # goal
        for idx, pose in tqdm(enumerate(pos)):
            if idx+goal_steps < len(pos):
                goal = transform_pose(pos[idx+goal_steps], pose)
                goals.append(goal)
        return acs, pos, goals
    else:
        return acs, pos

def transform_pose(pose, base_pose):
    x = pose[0] - base_pose[0]
    y = pose[1] - base_pose[1]
    yaw = pose[2] - base_pose[2]
    trans_pose = np.array([ x*np.cos(base_pose[2]) + y*np.sin(base_pose[2]),
                           -x*np.sin(base_pose[2]) + y*np.cos(base_pose[2]),
                           np.arctan2(np.sin(yaw), np.cos(yaw))])
    return trans_pose

def convert2lidar(sample_data):
    lidar = []
    for data in tqdm(sample_data):
        lidar.append(np.array(data.ranges))
    return lidar

def convert2imu(sample_data):
    imu = []
    for data in tqdm(sample_data):
        # imu
        imu_data = np.array([
            data.linear_acceleration.x,
            data.linear_acceleration.y,
            data.linear_acceleration.z,
            data.angular_velocity.x,
            data.angular_velocity.y,
            data.angular_velocity.z])
        imu.append(imu_data)
    return imu
