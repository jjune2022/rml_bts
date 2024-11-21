"""
#Date: 11.17.2024

local planner(publish the all waypoints on local planner())
publish(cx,cy,cyaw) - waypoints gone(5 points) + from closest_idx to car to closest_idx(1 points) + waypoints to go(9 points)
-c:course
you can select the path type
"""




import rospy
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry, Path
import numpy as np
import math
import sys
sys.path.append('/home/rml/scout_sim/src/erp-cml/path/')
from CubicSpline import cubic_spline_planner
# parameter
dl =0.1 # distance of interpolateed points
N_IND_SEARCH = 10  # Search index number
sp = 2.0 # constant linear velocity,, !!!!!!!!!!!!!!if change the value, you should change the 'linear velocity' on control node!!!!!!!!!!!!!!!!!!
NZ = 3 # # of state vector z = x,y,yaw
NU = 2 # # of input vector u = v,w
T = 15  # prediction horizon length
hz = 10 # callback rate
dt = 1 / hz # time step

class Pathplanner:
    def __init__(self):
        rospy.init_node('Pathplanner')
        rospy.Subscriber("/odom", Odometry, self.odom_update)
        self.marker_pub = rospy.Publisher('/transformed_points_marker', Marker, queue_size=10)
        self.path_pub = rospy.Publisher('/way_points', Path, queue_size=10)
        self.marker_id = 0  # Initialize marker ID
        self.odom_pose = None
        self.odom_twist = None
        self.x0 = 0
        self.y0 = 0
        self.previous_idx = 0

        ### generate path for 4cases , ck: curvature
        #self.cx, self.cy, self.cyaw, self.ck = get_straight_course(dl)
        #self.cx, self.cy, self.cyaw, self.ck = get_straight_course1(dl)
        #self.cx, self.cy, self.cyaw, self.ck = get_forward_course(dl)
        #self.cx, self.cy, self.cyaw, self.ck = get_switch_back_course(dl)
        #self.cx, self.cy, self.cyaw, self.ck = get_sine_course(dl)
        self.cx, self.cy, self.cyaw, self.ck = get_infinite_course(dl)



    def odom_update(self, data):
        self.odom_pose = data.pose.pose
        self.odom_twist = data.twist.twist
        self.x0, self.y0 = self.odom_pose.position.x, self.odom_pose.position.y


    def publish_way_points_marker(self):
        """
        Publish the waypoints to RViz as a Marker
        """
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "transformed_points"
        marker.id = self.marker_id  # Unique ID for the line strip
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        # Set the scale of the line
        marker.scale.x = 0.1  # Line thickness

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 0.3
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Add points to the marker
        for x, y in zip(self.cx, self.cy):
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)





        # Publish the marker
        self.marker_pub.publish(marker)
        self.marker_id += 1  # Increment the marker ID for the next marker

    def publish_way_points_path(self):
        """
        Publish the waypoints to RViz as a Path
        """
        path = Path()
        path.header.frame_id = "odom"
        path.header.stamp = rospy.Time.now()

        zref , idx = calc_ref_trajectory(self.x0, self.y0, self.cx, self.cy,
                                    self.cyaw, sp, dl, self.previous_idx)
        self.previous_idx = idx




        print(len(zref[0,:]))
        # Add points to the path
        for i in range(T):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "odom"
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose.position.x = zref[0, i]
            pose_stamped.pose.position.y = zref[1, i]
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = zref[2, i]
            path.poses.append(pose_stamped)

        # Publish the path
        self.path_pub.publish(path)


########################################### utils ###############################################
# calculation for nearest index
def calc_nearest_index(x0,y0, cx, cy, cyaw, previous_idx):
    """
    Simulation

    cx: course x position list
    cy: course y position list
    cy: course yaw position list
    cyaw: course yaw
    pind: closest waypoint

    state vector z = x,y,yaw
    """
    closest_idx = 0
    # Based on previous closest_idx(previous_idx) +- 20 points(becausoe of cross path problem)
    search_range = range(max(0, previous_idx - 20), min(len(cx), previous_idx+ 20))
    min_dist =float('inf')

    for i in search_range:

        dist = math.sqrt((x0 - cx[i]) ** 2 + (y0 - cy[i]) ** 2)
        if dist < min_dist:
            min_dist = dist
            closest_idx = i

    return closest_idx, min_dist

def calc_ref_trajectory(x0, y0, cx, cy, cyaw, sp, dl, previous_idx):
    """
    Simulation

    cx: course x position list
    cy: course y position list
    cy: course yaw position list
    ck: course curvature list
    sp: speed profile
    dl: course tick [m]

    """
    zref = np.zeros((NZ, T))
    ncourse = len(cx)

    ind, _ = calc_nearest_index(x0,y0, cx, cy, cyaw, previous_idx)

    # state vector z = x,y,yaw
    zref[0, 0] = cx[ind]
    zref[1, 0] = cy[ind]
    zref[2, 0] = cyaw[ind]


    travel = 0.0



    for i in range(T):
        ## when sp(velocity) is high
        #travel += sp * dt
        #dind = int(round(travel / dl))

        ## whensp is low
        dind = 10

        if ind > 4 and (ind + dind) < ncourse: #10< ind < len(cx)
            zref[0, i] = cx[i+ind - 5]
            zref[1, i] = cy[i+ind -5]
            zref[2, i] = cyaw[i+ind -5]


        elif (ind + dind) < ncourse:
            zref[0, i] = cx[i]
            zref[1, i] = cy[i]
            zref[2, i] = cyaw[i]


        else:
            zref[0, i] = cx[-1]
            zref[1, i] = cy[-1]
            zref[2, i] = cyaw[-1]


    return zref, ind







####################################################################################################





########################################### path type###############################################
def get_straight_course(dl):
    ax = [0.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    ay = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck


def get_straight_course1(dl):
    ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
    ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    cyaw = [i - math.pi for i in cyaw]

    return cx, cy, cyaw, ck


def get_forward_course(dl):
    ax = [0.0, 60.0, 125.0, 50.0, 75.0, 30.0, -10.0]
    ay = [0.0, 0.0, 50.0, 65.0, 30.0, 50.0, -20.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck


def get_straight_course(dl):
    """직선 경로 생성"""
    ax = [0.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    ay = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=dl)
    return cx, cy, cyaw, ck

def get_sine_course(dl):
    """사인파 경로 생성"""
    ax = np.arange(0, 50, 1.0)
    ay = 3.0 * np.sin(ax / 3.0)
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=dl)
    return cx, cy, cyaw, ck

def get_infinite_course(dl):
    """무한대(∞) 모양 경로 생성"""
    ax = []
    ay = []

    # 무한대 모양을 파라메트릭 방정식으로 생성
    t = np.linspace(0, 2*np.pi, 63)  # 63개 포인트로 설정하여 시작점과 끝점이 (0,0)이 되도록 함
    a = 10  # 가로 크기
    b = 5   # 세로 크기

    # 리사주 곡선을 이용한 무한대 모양
    x = a * np.sin(t)
    y = b * np.sin(2*t)
    ax = x.tolist()
    ay = y.tolist()

    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=dl)
    return cx, cy, cyaw, ck



# def get_switch_back_course(dl):
#     ax = [0.0, 30.0, 6.0, 20.0, 35.0]
#     ay = [0.0, 0.0, 20.0, 35.0, 20.0]
#     cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
#         ax, ay, ds=dl)
#     ax = [35.0, 10.0, 0.0, 0.0]
#     ay = [20.0, 30.0, 5.0, 0.0]
#     cx2, cy2, cyaw2, ck2, s2 = cubic_spline_planner.calc_spline_course(
#         ax, ay, ds=dl)
#     cyaw2 = [i - math.pi for i in cyaw2]
#     cx.extend(cx2)
#     cy.extend(cy2)
#     cyaw.extend(cyaw2)
#     ck.extend(ck2)

#     return cx, cy, cyaw, ck

def get_switch_back_course(dl):
    ax = [0.0, 3.0, 6.0]
    ay = [0.0, 3.0, -5.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    ax = [35.0, 10.0, 0.0, 0.0]
    ay = [20.0, 30.0, 5.0, 0.0]
    cx2, cy2, cyaw2, ck2, s2 = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    cyaw2 = [i - math.pi for i in cyaw2]
    cx.extend(cx2)
    cy.extend(cy2)
    cyaw.extend(cyaw2)
    ck.extend(ck2)

    return cx, cy, cyaw, ck



####################################################################################################

if __name__ == '__main__':
    path_planner = Pathplanner()
    rate = rospy.Rate(hz)  # 50 Hz
    while not rospy.is_shutdown():
        path_planner.publish_way_points_marker()
        path_planner.publish_way_points_path()
        rate.sleep()
