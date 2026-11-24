#!/usr/bin/env python3
import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from psr_hero.utils import V_MAX, W_MAX, clamp, pose_from_odom, wrap_angle

FIELD_L, FIELD_W = 1.8, 1.2
GOAL_W = 0.40
TEAMS = {"blue": (0, +1), "red": (3, -1)}

X_LIM = FIELD_L / 2.0 - 0.06
Y_LIM = FIELD_W / 2.0 - 0.06
GOAL_X = FIELD_L / 2.0
GOAL_HALF = GOAL_W / 2.0

K_V = 2.2
K_W = 4.0
GK_X = -0.82


class Soccer:
    def __init__(self):
        rospy.init_node("soccer")
        team = rospy.get_param("~team", "blue")
        self.first, self.attack = TEAMS[team]
        self.ids = [self.first + k for k in range(3)]
        self.poses = {}
        self.ball = None
        self.ball_vel = (0.0, 0.0)
        self.striker = 1

        self.pubs = {i: rospy.Publisher("/hero_%d/cmd_vel" % i, Twist, queue_size=1) for i in self.ids}

        for i in range(6):
            rospy.Subscriber("/hero_%d/odom" % i, Odometry, self.on_odom, callback_args=i, queue_size=1)
        rospy.Subscriber("/ball/odom", Odometry, self.on_ball, queue_size=1)

    def on_odom(self, msg, i):
        self.poses[i] = pose_from_odom(msg)

    def on_ball(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.ball = (p.x, p.y)
        self.ball_vel = (v.x, v.y)

    def drive(self, me, dx, dy, speed):
        err = wrap_angle(math.atan2(dy, dx) - me[2])
        if abs(err) > math.pi / 2:
            err = wrap_angle(err - math.pi)
            speed = -speed
        return speed * math.cos(err), clamp(K_W * err, -W_MAX, W_MAX)

    def go_to(self, me, tx, ty, mates=(), full=False):
        tx = clamp(tx, -X_LIM, X_LIM)
        ty = clamp(ty, -Y_LIM, Y_LIM)
        dx, dy = tx - me[0], ty - me[1]
        dist = math.hypot(dx, dy)
        if not full and dist < 0.025:
            return 0.0, 0.0

        for mx, my, _ in mates:
            ax, ay = me[0] - mx, me[1] - my
            r = math.hypot(ax, ay)
            if 1e-3 < r < 0.14:
                push = 0.06 * (0.14 - r) / r
                dx += ax * push
                dy += ay * push

        speed = V_MAX if full else min(V_MAX, K_V * dist)
        return self.drive(me, dx, dy, speed)

    def goalkeeper(self, me, ball, bvel):
        bx, by = ball
        bvx, bvy = bvel
        if bx < -0.70 and abs(by) < 0.20 and bvx > -0.03 and me[0] < bx - 0.01:
            return self.drive(me, bx + 0.10 - me[0], by - me[1], V_MAX)

        ty = by
        if bvx < -0.04 and bx > GK_X:
            dt = min((bx - GK_X) / (-bvx), 1.5)
            ty = by + bvy * dt
        ty = clamp(ty, -(GOAL_HALF - 0.03), GOAL_HALF - 0.03)

        if math.hypot(GK_X - me[0], ty - me[1]) < 0.02:
            err = wrap_angle(math.pi / 2 - me[2])
            if abs(err) > math.pi / 2:
                err = wrap_angle(err - math.pi)
            return 0.0, clamp(K_W * err, -W_MAX, W_MAX)

        return self.go_to(me, GK_X, ty)

    def play_striker(self, me, ball, bvel, opps, mates):
        d_now = math.hypot(ball[0] - me[0], ball[1] - me[1])
        lead = min(0.25, d_now / 0.40)
        bx = clamp(ball[0] + bvel[0] * lead, -GOAL_X, GOAL_X)
        by = clamp(ball[1] + bvel[1] * lead, -FIELD_W / 2.0, FIELD_W / 2.0)

        aim_x = GOAL_X + 0.05 if (bx < 0.78 or abs(by) < GOAL_HALF) else GOAL_X - 0.12
        aim_y = clamp(by * 0.35, -0.10, 0.10)
        if bx > 0.20 and abs(by) < 0.35:
            blockers = [o for o in opps if o[0] > bx + 0.04 and o[0] > 0.45 and abs(o[1]) < GOAL_HALF]
            if blockers:
                blk = min(blockers, key=lambda o: abs(o[1] - aim_y))
                aim_y = -0.14 if blk[1] >= 0.0 else 0.14

        gx, gy = aim_x - bx, aim_y - by
        gd = max(math.hypot(gx, gy), 1e-6)
        ux, uy = gx / gd, gy / gd

        rx, ry = me[0] - bx, me[1] - by
        along = rx * ux + ry * uy
        cross = -rx * uy + ry * ux

        behind_x = clamp(bx - 0.09 * ux, -X_LIM, X_LIM)
        behind_y = clamp(by - 0.09 * uy, -Y_LIM, Y_LIM)
        at_behind = math.hypot(behind_x - me[0], behind_y - me[1]) < 0.045

        if (along < -0.02 and abs(cross) < 0.055 and d_now < 0.28) or (at_behind and along < -0.01):
            return self.drive(me, bx + 0.12 * ux - me[0], by + 0.12 * uy - me[1], V_MAX)

        if along > 0.01 and math.hypot(rx, ry) < 0.20:
            side = 1.0 if cross >= 0 else -1.0
            if abs(by + side * 0.14 * ux) > Y_LIM:
                side = -side
            tx = bx - 0.12 * ux - side * 0.14 * uy
            ty = by - 0.12 * uy + side * 0.14 * ux
            return self.go_to(me, tx, ty, mates, full=True)

        return self.go_to(me, behind_x, behind_y, mates, full=True)

    def play_support(self, me, ball, striker_pose, mates):
        bx, by = ball
        if bx <= -0.15:
            tx = clamp(bx - 0.25, -0.62, -0.20)
            ty = clamp(by * 0.6, -0.26, 0.26)
        else:
            tx = clamp(bx - 0.22, -0.35, 0.50)
            side = -1.0 if striker_pose[1] >= 0.0 else 1.0
            ty = clamp(side * 0.20, -0.35, 0.35)
        return self.go_to(me, tx, ty, mates)

    def pick_striker(self, team_poses, ball):
        bx, by = ball
        costs = []
        for idx in (1, 2):
            x, y, _ = team_poses[idx]
            c = math.hypot(x - (bx - 0.09), y - by)
            if x > bx:
                c += 0.25
            if idx == self.striker:
                c -= 0.08
            costs.append((c, idx))
        self.striker = min(costs)[1]
        return self.striker

    def compute_commands(self, poses, ball):
        # normaliza para o lado azul (rotacao de 180 graus quando vermelho)
        s = self.attack
        blue = s > 0
        team = []
        for i in self.ids:
            x, y, yaw = poses[i]
            team.append((s * x, s * y, yaw if blue else wrap_angle(yaw + math.pi)))
        opps = [(s * poses[i][0], s * poses[i][1]) for i in range(6) if i not in self.ids]
        b = (s * ball[0], s * ball[1])
        bv = (s * self.ball_vel[0], s * self.ball_vel[1])

        st = self.pick_striker(team, b)
        sp = 3 - st

        cmds = [None, None, None]
        cmds[0] = self.goalkeeper(team[0], b, bv)
        cmds[st] = self.play_striker(team[st], b, bv, opps, (team[0], team[sp]))
        cmds[sp] = self.play_support(team[sp], b, team[st], (team[0], team[st]))
        return cmds

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
