#!/usr/bin/env python3
import math

import rospy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from psr_hero.utils import V_MAX, W_MAX, clamp, pose_from_odom, wrap_angle

BASE_X = 0.62
WALL = 0.8
HOME = (-BASE_X, 0.0)

K_W = 4.0
K_V = 2.0
K_HOLD = 20.0
HOLD_DIST = 0.015

R_THREAT = 0.35
K_REP = 0.32
WALL_DIST = 0.12
K_WALL = 0.08


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
        self.around = 0

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

    def drive(self, me, dx, dy, speed):
        err = wrap_angle(math.atan2(dy, dx) - me[2])
        if abs(err) > math.pi / 2:
            err = wrap_angle(err - math.pi)
            speed = -speed
        return speed * math.cos(err), clamp(K_W * err, -W_MAX, W_MAX)

    def go_to(self, me, x, y):
        dx, dy = x - me[0], y - me[1]
        return self.drive(me, dx, dy, min(V_MAX, K_V * math.hypot(dx, dy)))

    def attack(self, me, opps, enemy_flag, carrying):
        gx, gy = HOME if carrying else enemy_flag
        dx, dy = gx - me[0], gy - me[1]
        d = max(math.hypot(dx, dy), 1e-6)
        ux, uy = dx / d, dy / d
        fx, fy = ux, uy

        near = False
        for ox, oy in opps:
            if ox < -0.05:
                continue
            ax, ay = me[0] - ox, me[1] - oy
            r = max(math.hypot(ax, ay), 1e-3)
            if r > R_THREAT:
                continue
            ax, ay = ax / r, ay / r
            k = K_REP * (1.0 / r - 1.0 / R_THREAT)
            fx += k * ax
            fy += k * ay
            if -(ax * ux + ay * uy) > 0.0:
                near = True
                if self.around == 0:
                    self.around = 1 if (-ay * ux + ax * uy) >= 0 else -1
                fx += k * (-self.around * ay)
                fy += k * (self.around * ax)
        if not near:
            self.around = 0

        for i, f in ((0, 1.0), (1, 1.0), (0, -1.0), (1, -1.0)):
            gap = max(WALL - f * me[i], 0.01)
            if gap < WALL_DIST:
                push = K_WALL * (1.0 / gap - 1.0 / WALL_DIST)
                if i == 0:
                    fx -= f * push
                else:
                    fy -= f * push

        return self.drive(me, fx, fy, V_MAX)

    def defend(self, me, opps, own_flag):
        hx, hy = HOME
        if math.hypot(own_flag[0] - hx, own_flag[1] - hy) > 0.03:
            if own_flag[0] < -0.06:
                return self.drive(me, own_flag[0] - me[0], own_flag[1] - me[1], V_MAX)
            return self.go_to(me, -0.06, own_flag[1])

        ex, ey = hx - me[0], hy - me[1]
        if math.hypot(ex, ey) > HOLD_DIST:
            return self.go_to(me, hx, hy)

        ox, oy = min(opps, key=lambda o: math.hypot(o[0] - me[0], o[1] - me[1]))
        err = wrap_angle(math.atan2(oy - me[1], ox - me[0]) - me[2])
        if abs(err) > math.pi / 2:
            err = wrap_angle(err - math.pi)
        v = K_HOLD * (ex * math.cos(me[2]) + ey * math.sin(me[2]))
        return clamp(v, -V_MAX, V_MAX), clamp(K_W * err, -W_MAX, W_MAX)

    def compute_command(self, me, opponents, enemy_flag, own_flag, carrying):
        # normaliza para o lado azul (rotacao de 180 graus quando vermelho)
        s = self.side
        me = (s * me[0], s * me[1], me[2] if self.blue else wrap_angle(me[2] + math.pi))
        opps = [(s * o[0], s * o[1]) for o in opponents]
        enemy_flag = (s * enemy_flag[0], s * enemy_flag[1])
        own_flag = (s * own_flag[0], s * own_flag[1])
        if self.role == "defender":
            return self.defend(me, opps, own_flag)
        return self.attack(me, opps, enemy_flag, carrying)

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
