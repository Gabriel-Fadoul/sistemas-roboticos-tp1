#!/usr/bin/env python3
import math

import rospy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from psr_hero.utils import V_MAX, W_MAX, clamp, pose_from_odom, wrap_angle

BASE_X = 0.62
HOME = (-BASE_X, 0.0)

K_W = 3.0
K_V = 1.5


class CaptureTheFlag:
    def __init__(self):
        rospy.init_node("ctf")
        self.robot = rospy.get_param("~robot", 0)
        self.team = rospy.get_param("~team", "blue" if self.robot == 0 else "red")
        self.role = rospy.get_param("~role", "attacker")
        self.opponents = list(rospy.get_param("~opponents", [1 if self.robot == 0 else 0]))
        self.blue = self.team == "blue"
        self.side = 1.0 if self.blue else -1.0
        self.me = None
        self.opp = {}
        self.enemy_flag = None
        self.own_flag = None
        self.carrying = False

        self.pub = rospy.Publisher("/hero_%d/cmd_vel" % self.robot, Twist, queue_size=1)

        own, enemy = ("blue", "red") if self.blue else ("red", "blue")
        rospy.Subscriber("/hero_%d/odom" % self.robot, Odometry, self.on_me, queue_size=1)
        for k in self.opponents:
            rospy.Subscriber("/hero_%d/odom" % k, Odometry, self.on_opp, callback_args=k, queue_size=1)
        rospy.Subscriber("/flag/" + enemy, PointStamped, self.on_enemy_flag, queue_size=1)
        rospy.Subscriber("/flag/" + own, PointStamped, self.on_own_flag, queue_size=1)
        rospy.Subscriber("/hero_%d/has_flag" % self.robot, Bool, self.on_has_flag, queue_size=1)

    def on_me(self, msg):
        self.me = pose_from_odom(msg)

    def on_opp(self, msg, k):
        self.opp[k] = pose_from_odom(msg)

    def on_enemy_flag(self, msg):
        self.enemy_flag = (msg.point.x, msg.point.y)

    def on_own_flag(self, msg):
        self.own_flag = (msg.point.x, msg.point.y)

    def on_has_flag(self, msg):
        self.carrying = msg.data

    def go_to(self, me, x, y):
        dx, dy = x - me[0], y - me[1]
        if math.hypot(dx, dy) < 0.02:
            return 0.0, 0.0
        err = wrap_angle(math.atan2(dy, dx) - me[2])
        v = min(V_MAX, K_V * math.hypot(dx, dy)) * max(0.0, math.cos(err))
        return v, clamp(K_W * err, -W_MAX, W_MAX)

    def compute_command(self, me, opponents, enemy_flag, own_flag, carrying):
        s = self.side
        me = (s * me[0], s * me[1], me[2] if self.blue else wrap_angle(me[2] + math.pi))
        opps = [(s * o[0], s * o[1]) for o in opponents]
        enemy_flag = (s * enemy_flag[0], s * enemy_flag[1])

        if self.role == "defender":
            intruders = [o for o in opps if o[0] < 0.0]
            if intruders:
                ox, oy = min(intruders, key=lambda o: math.hypot(o[0] - HOME[0], o[1] - HOME[1]))
                return self.go_to(me, min(ox, -0.06), oy)
            return self.go_to(me, HOME[0] + 0.15, HOME[1])

        gx, gy = HOME if carrying else enemy_flag
        return self.go_to(me, gx, gy)

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if (self.me is None or len(self.opp) < len(self.opponents)
                    or self.enemy_flag is None or self.own_flag is None):
                rospy.logwarn_throttle(2.0, "waiting for odometry and flags")
            else:
                v, w = self.compute_command(self.me, list(self.opp.values()), self.enemy_flag,
                                            self.own_flag, self.carrying)
                cmd = Twist()
                cmd.linear.x, cmd.angular.z = v, w
                self.pub.publish(cmd)
            rate.sleep()


if __name__ == "__main__":
    CaptureTheFlag().run()
