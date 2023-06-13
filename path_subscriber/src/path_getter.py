#!/usr/bin/python3
import rospy 
from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import PoseStamped
import numpy as np

# create a node for getting the plan
rospy.init_node('get_plan_client')

# wait for the service which makes plans without acting on them
rospy.wait_for_service('/move_base/make_plan')

# get plan service
get_plan_service = rospy.ServiceProxy('/move_base/make_plan', GetPlan)

# initialize start
start = PoseStamped()
start.header.seq = 0
start.header.frame_id = "map" 
start.header.stamp = rospy.Time(0)
start.pose.position.x = 0.0
start.pose.position.y = 0.0

# initialize goal 1
Goal = PoseStamped()
Goal.header.seq = 0
Goal.header.frame_id = "map"
Goal.header.stamp = rospy.Time(0)

Goal.pose.position.x = 5.0
Goal.pose.position.y = 6.0

# Initialize goal 2
Goal2 = PoseStamped()
Goal2.header.seq = 0
Goal2.header.frame_id = "map"
Goal2.header.stamp = rospy.Time(0)
Goal2.pose.position.x = -3
Goal2.pose.position.y = 4


def create_pose(header_seq=0, frame_id="map", timestamp=rospy.Time(0), x=0.0,y=0.0):
    """
    Creates a PoseStamped for use in path planning
    
    geometry_msgs/PoseStamped Message
    A Pose with reference coordinate frame and timestamp
        Header header
        Pose pose
    """
    new_pose = PoseStamped()
    new_pose.header.seq = 0
    new_pose.header.frame_id = frame_id
    new_pose.header.stamp = timestamp
    new_pose.pose.position.x = x
    new_pose.pose.position.y = y
    return new_pose

def create_req(start_pose, goal_pose, tolerance=1.5):
    """
    Creates a plan request from a start, goal, and tolerance
    Exists in python docs, very poorly documented on site

    returns a GetPlanRequest
    """
    req = GetPlanRequest()
    req.start = start_pose
    req.goal = goal_pose
    req.tolerance = tolerance
    return req

def dist(p1, p2):
    """
    Simple calculation of euclidean distance between two x,y points
    
    Returns a number
    """
    x1,y1 = p1
    x2,y2 = p2
    return np.sqrt(np.square(x2 - x1) + np.square(y2 - y1))

def calc_plan_len(plan):
    """
    Calculates the length of the path by summation of distance between adjacent pose positions
    
    nav_msgs/Path Message
    An array of poses that represents a Path for a robot to follow
        Header header
        geometry_msgs/PoseStamped[] poses
    """
    # cheater lambda for extracting the x,y coordinate from the stamped pose
    pose2point = lambda spose: (spose.pose.position.x, spose.pose.position.y)

    poses = plan.poses
    total_dist = 0
    # iterate over all pairs
    for i in range(1, len(poses)):
        p1 = pose2point(poses[i-1])
        p2 = pose2point(poses[i])
        d = dist(p1,p2)
        total_dist += d
    
    return total_dist

def get_plan_between_poses(pose1, pose2):
    """
    Wrapper function for getting a plan between two stamped poses
    """
    req = create_req(pose1, pose2)
    resp = get_plan_service(req)
    return resp.plan

req = create_req(start, Goal)
req2 = create_req(Goal, Goal2)

resp = get_plan_service(req)
resp2 = get_plan_service(req2)

plan = resp.plan
plan2 = resp2.plan

print(calc_plan_len(plan))
print(calc_plan_len(plan2))
