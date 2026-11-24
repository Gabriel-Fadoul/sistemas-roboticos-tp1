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
        self.pub = rospy.Publisher(
            "/hero_%d/cmd_vel" % self.robot,
            Twist,
            queue_size=1
        )

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

        # LOG: placar publicado pelo juiz e ultimo estado impresso
        self.ultimo_placar = None
        self.ultimo_estado = None
        rospy.Subscriber(
            "/game/state",
            String,
            self.on_game,
            queue_size=1
        )

    def on_me(self, msg):
        self.me = pose_from_odom(msg)
        self.speed = msg.twist.twist.linear.x

    def on_opp(self, msg):
        self.opp = pose_from_odom(msg)

    def on_game(self, msg):
        # LOG: imprime o placar sempre que ele muda
        try:
            dados = json.loads(msg.data)

            placar = "wins=%s draws=%s" % (
                dados.get("wins"),
                dados.get("draws")
            )

            if placar != self.ultimo_placar:
                self.ultimo_placar = placar

                rospy.loginfo(
                    "PLACAR hero_%d: %s",
                    self.robot,
                    placar
                )

        except Exception:
            pass

    def compute_command(self, me, opp, speed, now):
        """Choose (linear_x, angular_z).

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

        # Parametros de seguranca
        safe_radius = RING_RADIUS - 0.05
        prediction_time = 0.20

        # Previsao da posicao futura do robo
        future_x = x + speed * math.cos(yaw) * prediction_time
        future_y = y + speed * math.sin(yaw) * prediction_time

        future_radius = math.hypot(future_x, future_y)

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

        # Verifica se estamos atras do adversario
        behind = (
            (x - ox) * ux +
            (y - oy) * uy
        ) < -0.02

        # Verifica se estamos empurrando o adversario para fora
        pushing = (
            self.state == "push"
            and distance < 0.14
            and behind
            and opp_radius > radius
            and opp_radius > 0.32
        )

        # Recuperacao preventiva baseada na posicao futura
        if (
            radius >= safe_radius
            or future_radius >= safe_radius
            or (radius > 0.30 and not pushing)
        ):
            if self.state != "recover":
                self.state = "recover"
                self.state_time = now

        # Se estiver perto da borda, volta para o centro
        if self.state == "recover":

            if radius < 0.20:
                self.state = "approach"
                self.state_time = now

            else:
                angle = math.atan2(-y, -x)
                error = wrap_angle(angle - yaw)

                direction = 1.0

                # Usa marcha a re se for mais rapido
                # do que girar para o centro
                if abs(error) > math.pi / 2:
                    error = wrap_angle(error - math.pi)
                    direction = -1.0

                w = clamp(
                    self.kp * error,
                    -W_MAX,
                    W_MAX
                )

                # Recuperacao com velocidade progressiva
                v = (
                    direction
                    * V_MAX
                    * max(0.0, math.cos(error))
                )

                return v, w

        # Primeiro tenta se posicionar atras do adversario
        if self.state == "approach":

            target_x = ox - 0.10 * ux - 0.05 * uy
            target_y = oy - 0.10 * uy + 0.05 * ux

            target_radius = math.hypot(target_x, target_y)

            # Mantem o ponto de aproximacao dentro da arena
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

            # Aproximacao mais rapida e progressiva
            v = V_MAX * max(0.0, math.cos(error))

            # Verifica se chegou perto do ponto de ataque
            target_distance = math.hypot(
                target_x - x,
                target_y - y
            )

            # Ataca somente quando estiver
            # em uma posicao favoravel
            if behind and (
                target_distance < 0.05
                or (
                    distance < 0.18
                    and now - self.state_time > 1.0
                )
            ):
                self.state = "push"
                self.state_time = now

        # Empurra o adversario na direcao da borda
        elif self.state == "push":

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

            # Velocidade de ataque progressiva
            # Velocidade maxima quando estiver alinhado
            v = V_MAX * max(0.0, math.cos(error))

            # Se perder a vantagem ou o adversario escapar,
            # tenta outro ataque
            if not behind or (
                distance > 0.22
                and now - self.state_time > 0.5
            ):
                self.state = "approach"
                self.state_time = now
                v = 0.0

        else:
            return 0.0, 0.0

        # BARREIRA DE SEGURANCA
        

        if radius > 0.01:

            # Projecao da orientacao do robo
            # sobre a direcao radial da arena
            radial_direction = (
                x * math.cos(yaw)
                + y * math.sin(yaw)
            ) / radius

            # Velocidade na direcao da borda
            radial_speed = v * radial_direction

            if radial_speed > 0.0:

                remaining = max(
                    0.0,
                    safe_radius - radius
                )

                max_radial_speed = (
                    remaining / prediction_time
                )

                # Reduz a velocidade caso o movimento
                # possa levar o robo para fora
                if radial_speed > max_radial_speed:
                    v *= max_radial_speed / radial_speed

        return v, w

    def run(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            if self.me is None or self.opp is None:
                rospy.logwarn_throttle(
                    2.0,
                    "waiting for odometry"
                )

            else:
                v, w = self.compute_command(
                    self.me,
                    self.opp,
                    self.speed,
                    rospy.get_time()
                )

                # LOG: imprime cada troca de estado,
                # com a distancia ao centro e ao adversario
                if self.state != self.ultimo_estado:

                    r_eu = math.hypot(
                        self.me[0],
                        self.me[1]
                    )

                    r_opp = math.hypot(
                        self.opp[0],
                        self.opp[1]
                    )

                    dist = math.hypot(
                        self.opp[0] - self.me[0],
                        self.opp[1] - self.me[1]
                    )

                    rospy.loginfo(
                        "hero_%d: %s -> %s | eu a %.2f m do centro, ele a %.2f m, distancia entre nos %.2f m",
                        self.robot,
                        self.ultimo_estado,
                        self.state,
                        r_eu,
                        r_opp,
                        dist
                    )

                    self.ultimo_estado = self.state

                cmd = Twist()
                cmd.linear.x, cmd.angular.z = v, w
                self.pub.publish(cmd)

            rate.sleep()


if __name__ == "__main__":
    Sumo().run()
