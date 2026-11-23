#!/usr/bin/env python3
"""Sumo (student template).

Push the other robot out of the circular ring (radius 0.40 m, centre (0, 0)).
A robot loses when its centre leaves the ring; a round ends in a draw after
40 s. There is no wall: use the odometry to know where you are.

Both robots have the same traction, so a head-on push is a stalemate. Think
about angles, timing and not leaving the ring yourself.

Topics:
  /hero_N/odom   nav_msgs/Odometry   pose of every robot
  /hero_N/laser  sensor_msgs/LaserScan   the 8-beam lidar also sees the opponent
  /hero_N/cmd_vel  geometry_msgs/Twist   your command

Run:  ./run.sh run sumo      (see README.md in this folder)
"""
import json
import math  # noqa: F401

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry  # noqa: F401
from std_msgs.msg import String

from psr_hero.utils import V_MAX, W_MAX, clamp, pose_from_odom, wrap_angle  # noqa: F401

RING_RADIUS = 0.40


class Sumo:
    def __init__(self):
        rospy.init_node("sumo")
        self.robot = rospy.get_param("~robot", 0)
        self.opponent = rospy.get_param("~opponent", 1)
        self.me = None  # (x, y, yaw)
        self.speed = 0.0  # own forward speed, m/s
        self.opp = None  # (x, y, yaw)

        # Estados da estrategia
        self.state = "approach"
        self.state_time = rospy.get_time()

        # Ganho do controlador angular
        self.kp = 3.0

        # Ready: the publisher for the velocity command.
        self.pub = rospy.Publisher("/hero_%d/cmd_vel" % self.robot, Twist, queue_size=1)

        # TODO 1 (ROS): subscribe to the odometry of your robot (self.robot) -> self.on_me
        #               and of the opponent (self.opponent) -> self.on_opp.

        rospy.Subscriber(
            "/hero_%d/odom" % self.robot,
            Odometry,
            self.on_me
        )

        rospy.Subscriber(
            "/hero_%d/odom" % self.opponent,
            Odometry,
            self.on_opp
        )

        # LOG: placar publicado pelo juiz e último estado impresso
        self.ultimo_placar = None
        self.ultimo_estado = None
        rospy.Subscriber("/game/state", String, self.on_game, queue_size=1)

    def on_me(self, msg):
        # TODO 1 (ROS): self.me = pose_from_odom(msg); self.speed = msg.twist.twist.linear.x
        self.me = pose_from_odom(msg)
        self.speed = msg.twist.twist.linear.x

    def on_opp(self, msg):
        # TODO 1 (ROS): self.opp = pose_from_odom(msg)
        self.opp = pose_from_odom(msg)

    def on_game(self, msg):
        # LOG: imprime o placar sempre que ele muda
        try:
            dados = json.loads(msg.data)
            placar = "wins=%s draws=%s" % (dados.get("wins"), dados.get("draws"))
            if placar != self.ultimo_placar:
                self.ultimo_placar = placar
                rospy.loginfo("PLACAR hero_%d: %s", self.robot, placar)
        except Exception:
            pass

    def compute_command(self, me, opp, speed, now):
        """TODO 2: choose (linear_x, angular_z).

        me, opp: (x, y, yaw). speed: own forward speed (m/s). now: time (s).
        """

        x, y, yaw = me
        ox, oy, _ = opp

        # Distancia ate o adversario
        dx = ox - x
        dy = oy - y
        distance = math.hypot(dx, dy)

        # Distancia do robo ate o centro
        radius = math.hypot(x, y)

        # Distancia do adversario ate o centro
        opp_radius = math.hypot(ox, oy)

        # Verifica se estamos empurrando o adversario para fora
        pushing = (
            self.state == "push"
            and distance < 0.14
            and opp_radius > radius
            and opp_radius > 0.32
        )

        # Continua o ataque se estiver em uma posicao favoravel
        if radius > 0.36 or (radius > 0.30 and not pushing):
            self.state = "recover"

        # Se estiver perto da borda, volta para o centro
        if self.state == "recover":

            if radius < 0.20:
                self.state = "approach"
                self.state_time = now

            else:
                angle = math.atan2(-y, -x)
                error = wrap_angle(angle - yaw)

                w = clamp(
                    self.kp * error,
                    -W_MAX,
                    W_MAX
                )

                if abs(error) < 0.5:
                    v = V_MAX * 0.7
                else:
                    v = 0.0

                return v, w

        # Direcao do centro para o adversario
        if opp_radius > 0.01:
            ux = ox / opp_radius
            uy = oy / opp_radius

        elif distance > 0.01:
            ux = dx / distance
            uy = dy / distance

        else:
            ux = math.cos(yaw)
            uy = math.sin(yaw)

        # Primeiro tenta se posicionar atras do adversario
        if self.state == "approach":

            target_x = ox - 0.10 * ux - 0.05 * uy
            target_y = oy - 0.10 * uy + 0.05 * ux

            target_radius = math.hypot(target_x, target_y)

            if target_radius > 0.28:
                target_x *= 0.28 / target_radius
                target_y *= 0.28 / target_radius

            angle = math.atan2(
                target_y - y,
                target_x - x
            )

            error = wrap_angle(angle - yaw)

            w = clamp(
                self.kp * error,
                -W_MAX,
                W_MAX
            )

            if abs(error) < 0.5:
                v = V_MAX * 0.8
            else:
                v = 0.0

            # Verifica se chegou perto do ponto de ataque
            target_distance = math.hypot(
                target_x - x,
                target_y - y
            )

            # Tenta chegar ao ponto lateral antes de empurrar
            if target_distance < 0.05 or now - self.state_time > 1.5:
                self.state = "push"
                self.state_time = now

            return v, w

        # Empurra o adversario na direcao da borda
        if self.state == "push":

            target_x = ox + 0.05 * ux
            target_y = oy + 0.05 * uy

            angle = math.atan2(
                target_y - y,
                target_x - x
            )

            error = wrap_angle(angle - yaw)

            w = clamp(
                self.kp * error,
                -W_MAX,
                W_MAX
            )

            if abs(error) < 0.4:
                v = V_MAX

            elif abs(error) < 0.8:
                v = V_MAX * 0.4

            else:
                v = 0.0

            # Se o adversario escapar, tenta outro ataque
            if distance > 0.22 and now - self.state_time > 1.0:
                self.state = "approach"
                self.state_time = now

            return v, w

        return 0.0, 0.0

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.me is None or self.opp is None:
                rospy.logwarn_throttle(2.0, "waiting for odometry (TODO 1)")
            else:
                v, w = self.compute_command(self.me, self.opp, self.speed, rospy.get_time())

                # LOG: imprime cada troca de estado, com a distância ao centro e ao adversário
                if self.state != self.ultimo_estado:
                    r_eu = math.hypot(self.me[0], self.me[1])
                    r_opp = math.hypot(self.opp[0], self.opp[1])
                    dist = math.hypot(self.opp[0] - self.me[0], self.opp[1] - self.me[1])
                    rospy.loginfo("hero_%d: %s -> %s | eu a %.2f m do centro, ele a %.2f m, distancia entre nos %.2f m",
                                  self.robot, self.ultimo_estado, self.state, r_eu, r_opp, dist)
                    self.ultimo_estado = self.state
                cmd = Twist()
                cmd.linear.x, cmd.angular.z = v, w
                self.pub.publish(cmd)
            rate.sleep()


if __name__ == "__main__":
    Sumo().run()
