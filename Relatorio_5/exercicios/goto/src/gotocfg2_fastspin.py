#!/usr/bin/env python3
"""Go-to-goal navigation (student template).

Goal: drive the robot to the goal point, then to the next one the simulator
draws, as many times as possible. A goal counts when the robot centre gets
within 6 cm of it.

Topics:
  /hero_N/odom   nav_msgs/Odometry         pose in the 'world' frame (x, y, yaw)
  /hero_N/goal   geometry_msgs/PointStamped  current goal (world frame)
  /hero_N/cmd_vel  geometry_msgs/Twist      what you publish (linear.x, angular.z)

Run:  ./run.sh run goto      (see README.md in this folder)
"""
import csv
import math  # noqa: F401
import os
import time

import rospy
from geometry_msgs.msg import PointStamped, Twist  # noqa: F401
from nav_msgs.msg import Odometry  # noqa: F401

from psr_hero.utils import V_MAX, W_MAX, clamp, pose_from_odom, wrap_angle  # noqa: F401


class Goto:
    def __init__(self):
        # Inicializa o nó ROS e identifica qual robô será controlado.
        rospy.init_node("goto")
        self.robot = rospy.get_param("~robot", 0)

        # Pose e meta atuais. Estes valores iniciais são placeholders: se o loop
        # começar antes de chegarem mensagens, o robô pode seguir uma meta inválida.
        self.pose = None # (x, y, yaw)
        self.goal = None  # (x, y)
        self.goal_idx = 0
        self.last_goal = None

        filename = "gotocfg2_%s.csv" % time.strftime("%Y%m%d_%H%M%S", time.localtime())
        filepath = os.path.join(os.path.dirname(os.path.abspath(__file__)), filename)
        self.csv_file = open(filepath, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            ["t", "x", "y", "yaw", "gx", "gy", "goal_idx", "rho", "alpha", "v", "w"]
        )
        self.log_start_time = None
        rospy.on_shutdown(self.close_log)

        # Publica velocidades no tópico do robô selecionado.
        self.pub = rospy.Publisher("/hero_%d/cmd_vel" % self.robot, Twist, queue_size=1)

        # Recebe a pose do robô e a posição da meta; cada mensagem chama seu callback.
        self.odom_sub = rospy.Subscriber(
            "/hero_%d/odom" % self.robot, Odometry, self.on_odom, queue_size=1
        )
        self.goal_sub = rospy.Subscriber(
            "/hero_%d/goal" % self.robot, PointStamped, self.on_goal, queue_size=1
        )

    def on_odom(self, msg):
        # Extrai posição (x, y) e orientação yaw da mensagem de odometria.
        self.pose = pose_from_odom(msg)

    def on_goal(self, msg):
        # A meta é publicada no referencial do mundo; z não é necessário neste plano.
        goal = (msg.point.x, msg.point.y)
        if goal != self.last_goal:
            self.goal_idx += 1
            self.last_goal = goal
        self.goal = goal

    def compute_command(self, pose, goal):
        """Calcula velocidades linear e angular para alcançar a meta.

        pose contém (x, y, yaw), goal contém (x, y); o retorno é
        (linear_x, angular_z), em m/s e rad/s.
        """
        x, y, yaw = pose
        goal_x, goal_y = goal

        # Vetor e distância em linha reta entre o robô e a meta.
        dx = goal_x - x
        dy = goal_y - y
        distance = math.hypot(dx, dy)

        # O simulador considera a meta atingida dentro de um raio de 6 cm.
        if distance < 0.06:
            return 0.0, 0.0

        # Direção desejada menos orientação atual. wrap_angle escolhe o menor
        # erro equivalente entre -pi e pi, evitando uma volta longa desnecessária.
        heading_error = wrap_angle(math.atan2(dy, dx) - yaw)

        # Controle proporcional de orientação; clamp limita a rotação ao máximo
        # aceito pelo robô. O ganho 2.0 pode ser ajustado após observar o movimento.
        w = clamp(4.0 * heading_error, -W_MAX, W_MAX)

        # Avança mais quando está alinhado, mas mantém 25% da escala para não
        # ficar girando parado quando a meta está atrás. A distância também reduz
        # a velocidade perto da meta. V_MAX e clamp garantem o limite permitido.
        forward_scale = max(0.25, math.cos(heading_error))
        v = clamp(min(V_MAX, distance) * forward_scale, 0.0, V_MAX)

        # Possíveis melhorias: parametrizar ganhos/limites, desacelerar de forma
        # mais gradual perto da meta e ajustar o avanço mínimo para cada cenário.
        return v, w

    def log_data(self, pose, goal, goal_idx, v, w):
        x, y, yaw = pose
        gx, gy = goal
        dx = gx - x
        dy = gy - y
        rho = math.hypot(dx, dy)
        alpha = wrap_angle(math.atan2(dy, dx) - yaw)

        now = rospy.get_time()
        if self.log_start_time is None:
            self.log_start_time = now
        t = now - self.log_start_time

        self.csv_writer.writerow(
            [
                "%.3f" % t,
                "%.4f" % x,
                "%.4f" % y,
                "%.4f" % yaw,
                "%.4f" % gx,
                "%.4f" % gy,
                goal_idx,
                "%.4f" % rho,
                "%.4f" % alpha,
                "%.4f" % v,
                "%.4f" % w,
            ]
        )
        self.csv_file.flush()

    def close_log(self):
        if not self.csv_file.closed:
            self.csv_file.close()

    def run(self):
        # Executa o controlador a 20 Hz enquanto o nó ROS estiver ativo.
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            # Evita publicar comandos enquanto ainda faltam dados dos sensores.
            # Com os placeholders de __init__, esta proteção só atua após usar
            # None como valor inicial para pose e goal.
            pose = self.pose
            goal = self.goal
            goal_idx = self.goal_idx
            if pose is None or goal is None:
                rospy.logwarn_throttle(2.0, "waiting for pose and goal (TODO 1)")
            else:
                # Calcula as velocidades, preenche a mensagem Twist e publica.
                v, w = self.compute_command(pose, goal)
                cmd = Twist()
                cmd.linear.x, cmd.angular.z = v, w
                self.pub.publish(cmd)
                self.log_data(pose, goal, goal_idx, v, w)
            rate.sleep()


if __name__ == "__main__":
    # Ponto de entrada quando o arquivo é executado diretamente.
    Goto().run()
