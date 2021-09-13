#!/usr/bin/python
#-*- coding: utf-8 -*-
import rospy
import numpy as np
import pickle
import tf
import time

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Float64
from frenet_converter import get_frenet, get_cartesian
from polynomials import QuinticPolynomial, QuarticPolynomial

def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

def collision_check(fp, obs, mapx, mapy, maps):
    for i in range(len(obs[:, 0])):
        # get obstacle's position (x,y)
        obs_xy = get_cartesian( obs[i, 0], obs[i, 1], mapx, mapy, maps)

        d = [((_x - obs_xy[0]) ** 2 + (_y - obs_xy[1]) ** 2)
             for (_x, _y) in zip(fp.x, fp.y)]

        collision = any([di <= COL_CHECK ** 2 for di in d])

        if collision:
            return True

    return False

class FrenetPlanner(object):
    def __init__(self):
        self.rear_x = 0.0
        self.rear_y = 0.0
        self.yaw = 0.0
        self.v = 0.0

        self.acc_cmd = 0.0

        with open("/home/nvidia/xycar_ws/final_project/grepp_project_ws/src/map/path.pkl", "rb") as f:
            path = pickle.load(f)
            steps = 50
            last = 4701
            self.path = {
                'x': path['x'][:last:steps],
                'y': path['y'][:last:steps],
                'yaw': path['yaw'][:last:steps],
                's': [0.0]
            }

            for i in range(1, len(self.path['x'])):
                prev_s = self.path['s'][i-1]
                dx = self.path['x'][i] - self.path['x'][i-1]
                dy = self.path['y'][i] - self.path['y'][i-1]
                ds = np.hypot(dx, dy)
                self.path['s'].append(prev_s + ds)

        self.ego_pose_sub = rospy.Subscriber("ego_pose", PoseStamped, self.PoseCallBack)
        self.speed_sub = rospy.Subscriber("ego_speed", Float64, self.CurSpeedCallBack)
        self.acc_cmd_sub = rospy.Subscriber("acc_cmd", Float64, self.AccCmdCallBack)


    def PoseCallBack(self, msg):
        self.rear_x = msg.pose.position.x
        self.rear_y = msg.pose.position.y
        orientation_q = msg.pose.orientation
        _, _, self.yaw = tf.transformations.euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])

    def CurSpeedCallBack(self, msg):
        self.v = msg.data

    def AccCmdCallBack(self, msg):
        self.acc_cmd = msg.data

    def Plan(self, target_speed):
        ## 1. get initial state in frenet
        s0, d0 = get_frenet(self.rear_x, self.rear_y, self.path['x'], self.path['y'])

        s_init = np.array([0.0, self.v, self.acc_cmd])
        d_init = np.array([d0, 0.0, 0.0])

        ## 2. candidate conditions
        Ts = [2.0]
        s_term = np.array([target_speed, 0.0])

        # d_terms = [np.array([0.0, 0.0, 0.0])]
        d_terms = []
        for _d in [-1.5, -1.0, -0.5, -0.5, 1.0, 1.5]:
            d_terms.append(np.array([_d, 0.0, 0.0]))

        ## 3. longitudinal traj. candidates
        longi_candidates = []
        for T in Ts:
            longi_input = [s_init[0], s_init[1], s_init[2], s_term[0], s_term[1], T]
            longi_candidates.append(QuarticPolynomial(*longi_input))

        ## 4. lateral traj. candidates
        lat_candidates = []
        for T in Ts:
            for d_term in d_terms:
                lat_input = [d_init[0], d_init[1], d_init[2], d_term[0], d_term[1], d_term[2], T]
                lat_candidates.append(QuinticPolynomial(*lat_input))

        ## 5. Evaluate candidates
        longi_costs = []
        lat_costs = []

        for longi_candidate in longi_candidates:
            ts = np.arange(0.1, longi_candidate.T, 0.1)
            jerks = [longi_candidate.GetJerk(t) for t in ts]
            jerk_square = sum([j*j for j in jerks])

            longi_cost = jerk_square + longi_candidate.T + 0.0
            longi_costs.append(longi_cost)

        for lat_candidate in lat_candidates:
            ts = np.arange(0.1, lat_candidate.T, 0.1)
            jerks = [lat_candidate.GetJerk(t) for t in ts]
            jerk_square = sum([j*j for j in jerks])

            lat_cost = jerk_square + lat_candidate.T + np.abs(lat_candidate.xf)
            lat_costs.append(lat_cost)

        best_longi_idx = 0
        best_lat_idx = 0
        min_cost = 1e10
        for i in range(len(longi_costs)):
            for j in range(len(lat_costs)):
                total_cost = longi_costs[i] + lat_costs[j]
                if total_cost < min_cost:
                    min_cost = total_cost
                    best_longi_idx = i
                    best_lat_idx = j

        ## 6. Get best trajectory
        best_longi_traj = longi_candidates[best_longi_idx]
        best_lat_traj = lat_candidates[best_lat_idx]

        ## 7. Transform the best traj into Cartesian
        ts = np.arange(0.1, 2.0, 0.1)

        xs, ys, yaws = [], [], []
        for t in ts:
            s = best_longi_traj.GetPosition(t) + s0
            d = best_lat_traj.GetPosition(t)
            x, y, yaw = get_cartesian(
                s, d, self.path['x'], self.path['y'], self.path['s'])
            xs.append(x)
            ys.append(y)
            yaws.append(yaw)

        return xs, ys, yaws

    def GetPathMsg(self, xs, ys, yaws):
        path_msg = Path()
        path_msg.header.frame_id = '/map'
        path_msg.header.stamp = rospy.Time.now()

        path_msg.poses = []

        for x, y, yaw in zip(xs, ys, yaws):
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.frame_id = '/map'
            pose_stamped_msg.header.stamp = rospy.Time.now()
            pose_stamped_msg.pose.position.x = x
            pose_stamped_msg.pose.position.y = y
            pose_stamped_msg.pose.position.z = 0.5

            quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
            pose_stamped_msg.pose.orientation = Quaternion(*quat)
            path_msg.poses.append(pose_stamped_msg)

        return path_msg

    def PlanAndGetPathMsg(self, target_speed):
        xs, ys, yaws = self.Plan(target_speed)
        path_msg = self.GetPathMsg(xs, ys, yaws)
        return path_msg



if __name__ == '__main__':
    rospy.init_node("planner_node")
    trajectory_pub = rospy.Publisher("planner/best_trajectory", Path, queue_size=1)

    planner = FrenetPlanner()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        t0 = time.time()
        path_msg = planner.PlanAndGetPathMsg(20.0)
        t1 = time.time()
        print("t: %.1f ms" % ((t1-t0)*1e3))
        trajectory_pub.publish(path_msg)
        r.sleep()
