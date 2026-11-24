#!/usr/bin/env python3
import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from psr_hero.utils import V_MAX, W_MAX, clamp, pose_from_odom, wrap_angle

FIELD_L, FIELD_W = 1.8, 1.2
GOAL_W = 0.40
TEAMS = {"blue": (0, +1), "red": (3, -1)}

K_V = 1.2
K_W = 2.5


class Soccer:
    def __init__(self):
        rospy.init_node("soccer")
        team = rospy.get_param("~team", "blue")
        self.first, self.attack = TEAMS[team]
        self.ids = [self.first + k for k in range(3)]
        self.poses = {}
        self.ball = None

        self.pubs = {i: rospy.Publisher("/hero_%d/cmd_vel" % i, Twist, queue_size=1) for i in self.ids}

        for i in range(6):
            rospy.Subscriber("/hero_%d/odom" % i, Odometry, self.on_odom, callback_args=i, queue_size=1)
        rospy.Subscriber("/ball/odom", Odometry, self.on_ball, queue_size=1)

    def on_odom(self, msg, i):
        self.poses[i] = pose_from_odom(msg)

    def on_ball(self, msg):
        p = msg.pose.pose.position
        self.ball = (p.x, p.y)

    def go_to(self, me, tx, ty):
        dx, dy = tx - me[0], ty - me[1]
        dist = math.hypot(dx, dy)
        if dist < 0.03:
            return 0.0, 0.0
        err = wrap_angle(math.atan2(dy, dx) - me[2])
        v = min(V_MAX, K_V * dist) * max(0.0, math.cos(err))
        return v, clamp(K_W * err, -W_MAX, W_MAX)

    def compute_commands(self, poses, ball):
        s = self.attack
        blue = s > 0
        team = []
        for i in self.ids:
            x, y, yaw = poses[i]
            team.append((s * x, s * y, yaw if blue else wrap_angle(yaw + math.pi)))
        bx, by = s * ball[0], s * ball[1]

        # goleiro: acompanha o y da bola na linha do gol
        gk_cmd = self.go_to(team[0], -0.78, clamp(by, -0.18, 0.18))

        # atacante fixo: alinha atras da bola no eixo x e empurra para +x
        if team[1][0] < bx - 0.02 and abs(team[1][1] - by) < 0.08:
            st_cmd = self.go_to(team[1], bx + 0.10, by)
        else:
            st_cmd = self.go_to(team[1], clamp(bx - 0.12, -0.84, 0.84), clamp(by, -0.54, 0.54))

        # apoio fixo: fica no meio-campo acompanhando a bola
        sp_cmd = self.go_to(team[2], -0.30, clamp(by * 0.5, -0.35, 0.35))

        return [gk_cmd, st_cmd, sp_cmd]

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.ball is None or len(self.poses) < 6:
                rospy.logwarn_throttle(2.0, "waiting for odometry and ball")
            else:
                for i, (v, w) in zip(self.ids, self.compute_commands(self.poses, self.ball)):
                    cmd = Twist()
                    cmd.linear.x, cmd.angular.z = v, w
                    self.pubs[i].publish(cmd)
            rate.sleep()


if __name__ == "__main__":
    Soccer().run()
