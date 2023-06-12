#!/usr/bin/python3
import rospy 
#import srv
from nav_msgs.srv import GetPlan, GetPlanRequest
#from nav_msgs import srv
from geometry_msgs.msg import PoseStamped

import numpy as np

rospy.init_node('get_plan_client')
rospy.wait_for_service('/move_base/make_plan')

start = PoseStamped()
start.header.seq = 0
start.header.frame_id = "map"
start.header.stamp = rospy.Time(0)

start.pose.position.x = 0.0
start.pose.position.y = 0.0

Goal = PoseStamped()
Goal.header.seq = 0
Goal.header.frame_id = "map"
Goal.header.stamp = rospy.Time(0)

Goal.pose.position.x = 5.0
Goal.pose.position.y = 6.0

Goal2 = PoseStamped()
Goal2.header.seq = 0
Goal2.header.frame_id = "map"

Goal2.header.stamp = rospy.Time(0)

Goal2.pose.position.x = -3
Goal2.pose.position.y = 4

# get plan
get_plan_service = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
#req = create_req(start, Goal)
#req2 = create_req(start, Goal2)
#req = GetPlanRequest()
#req.start = start
#req.goal = Goal
#req.tolerance = 1.5

#resp = get_plan_service(req)
#resp2 = get_plan_service(req2)
#plan = resp.plan

def create_req(start, goal, tolerance=1.5):

    req = GetPlanRequest()
    req.start = start
    req.goal = goal
    req.tolerance = tolerance
    return req

def dist(p1, p2):
    x1,y1 = p1
    x2,y2 = p2
    return np.sqrt(np.square(x2 - x1) + np.square(y2 - y1))

def calc_plan_len(plan):
    pose2point = lambda spose: (spose.pose.position.x, spose.pose.position.y)
    poses = plan.poses
    total_dist = 0
    for i in range(1, len(poses)):
        p1 = pose2point(poses[i-1])
        p2 = pose2point(poses[i])
        d = dist(p1,p2)
        total_dist += d
    return total_dist

req = create_req(Goal2, Goal)
req2 = create_req(Goal, Goal2)
#req = GetPlanRequest()
#req.start = start
#req.goal = Goal
#req.tolerance = 1.5

resp = get_plan_service(req)
resp2 = get_plan_service(req2)
plan = resp.plan
plan2 = resp2.plan

print(calc_plan_len(plan))
print(calc_plan_len(plan2))
#resp = get_plan(req.start, req.goal, req.tolerance)
#print(resp)
