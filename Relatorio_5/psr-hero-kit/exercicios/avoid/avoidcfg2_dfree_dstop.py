#!/usr/bin/env python3
"""Obstacle avoidance with the 8-beam lidar (student template).

Goal: keep the robot moving in the arena without touching the obstacle in the
middle or the walls.

Lidar: /hero_N/laser (sensor_msgs/LaserScan), 8 beams, range 0.037 .. 0.5 m.
Beam k points at the angle -pi + k * pi/4 from the robot heading
(counter-clockwise positive), so beam 4 looks forward, beams 3 and 5 look 45
degrees to the right and to the left, and beam 0 looks backwards. A beam that
sees nothing reports 0.5. Distances are measured from the robot centre, so a
robot touching a wall reads about 0.037.

Run:  ./run.sh run avoid      (see README.md in this folder)
"""
import csv
import math
import os
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from psr_hero.utils import V_MAX, W_MAX, clamp, pose_from_odom


class Avoid:
    def __init__(self):
        # Cria o nó ROS chamado "avoid". Este nó será responsável por controlar o robô.
        rospy.init_node("avoid")

        # Lê o número do robô a partir do parâmetro ~robot; se não existir, usa 0.
        # Isso permite utilizar o mesmo código para diferentes robôs (hero_0, hero_1 etc.).
        self.robot = rospy.get_param("~robot", 0)

        # Guarda a última leitura do sensor lidar recebida.
        # Começa como None até que a primeira mensagem de LaserScan chegue.
        self.scan = None
        self.pose = None

        filename = "avoidcfg2_%s.csv" % time.strftime("%Y%m%d_%H%M%S", time.localtime())
        filepath = os.path.join(os.path.dirname(os.path.abspath(__file__)), filename)
        self.csv_file = open(filepath, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["t", "x", "y", "yaw", "min_range", "v", "w"])
        self.log_start_time = None
        rospy.on_shutdown(self.close_log)

        # Publicador da velocidade do robô: envia comandos do tipo geometry_msgs/Twist
        # para o tópico /hero_X/cmd_vel, onde X é o identificador do robô.
        self.pub = rospy.Publisher("/hero_%d/cmd_vel" % self.robot, Twist, queue_size=1)

        # Assinante do laser: recebe mensagens do tópico /hero_X/laser e chama on_scan
        # sempre que uma nova leitura do lidar chega.
        self.sub = rospy.Subscriber(
            "/hero_%d/laser" % self.robot, LaserScan, self.on_scan, queue_size=1
        )
        self.odom_sub = rospy.Subscriber(
            "/hero_%d/odom" % self.robot, Odometry, self.on_odom, queue_size=1
        )

    def on_scan(self, scan):
        # Callback do sensor lidar. Guarda a última leitura para que o loop principal
        # possa usá-la para decidir como o robô deve se mover.
        self.scan = scan

    def on_odom(self, msg):
        self.pose = pose_from_odom(msg)

    def compute_command(self, ranges):
        """Turn the 8 lidar ranges into a velocity command.

        ranges: list with 8 floats (m). Returns (linear_x [m/s], angular_z [rad/s]).
        Linear speed is limited to V_MAX = 0.2 m/s and angular speed to W_MAX = 4 rad/s.
        """
        # O sensor lidar envia 8 medidas, cada uma correspondente a um feixe.
        # Se o vetor não tiver exatamente 8 elementos, é melhor falhar cedo para não
        # gerar comportamento imprevisível.
        if len(ranges) != 8:
            raise ValueError("expected 8 lidar ranges")

        # O feixe 4 aponta para frente; os feixes 3 e 5 são diagonais para frente.
        # A ideia é detectar um obstáculo antes do robô vir a encostar diretamente nele.
        front = min(ranges[3:6])

        # left e right usam a média dos feixes laterais. Isso suaviza pequenas variações
        # e ajuda a decidir para que lado o robô deve virar.
        left = (ranges[5] + ranges[6]) / 2.0
        right = (ranges[3] + ranges[2]) / 2.0

        # Caso exista um obstáculo diretamente à frente e os lados estejam quase
        # equilibrados, a simetria faria o robô não girar. Ajustamos levemente a esquerda
        # para que ele comece a desviar antes de ficar preso.
        if front < 0.35 and abs(left - right) < 0.05:
            left += 0.15

        # Velocidade linear: depende da distância até o obstáculo frontal.
        # Quanto mais perto do obstáculo, menor a velocidade; o clamp limita o valor
        # entre 0 e 1 para manter a velocidade dentro do máximo permitido.
        v = V_MAX * clamp((front - 0.08) / (0.22 - 0.08), 0.0, 1.0)

        # Velocidade angular: a diferença entre left e right controla a direção do giro.
        # Se left > right, o robô gira para a direita; se left < right, gira para a esquerda.
        # O clamp mantém o valor dentro do limite W_MAX.
        w = clamp(3.0 * (left - right), -W_MAX, W_MAX)

        # Retorna a combinação de velocidade linear e angular que será usada pelo controle.
        return v, w

    def publish(self, v, w):
        # Cria a mensagem Twist, que carrega a velocidade linear e angular do robô.
        cmd = Twist()

        # linear.x controla o avanço do robô na direção frontal.
        cmd.linear.x = v

        # angular.z controla a rotação do robô no plano horizontal.
        cmd.angular.z = w

        # Envia o comando para o tópico de controle do robô.
        self.pub.publish(cmd)

    def log_data(self, pose, ranges, v, w):
        now = rospy.get_time()
        if self.log_start_time is None:
            self.log_start_time = now
        t = now - self.log_start_time

        if pose is None:
            x = y = yaw = math.nan
        else:
            x, y, yaw = pose

        self.csv_writer.writerow(
            [
                "%.3f" % t,
                "%.4f" % x,
                "%.4f" % y,
                "%.4f" % yaw,
                "%.4f" % min(ranges),
                "%.4f" % v,
                "%.4f" % w,
            ]
        )
        self.csv_file.flush()

    def close_log(self):
        if not self.csv_file.closed:
            self.csv_file.close()

    def run(self):
        # Define a taxa do loop principal em 20 Hz, ou seja, 20 iterações por segundo.
        rate = rospy.Rate(20)

        # Loop principal do comportamento de evasão.
        while not rospy.is_shutdown():
            # Se ainda não chegou o primeiro dado do lidar, avisa para o usuário.
            if self.scan is None:
                rospy.logwarn_throttle(2.0, "no lidar data yet: is the subscriber of TODO 1 in place?")
            else:
                # Converte a leitura do sensor para uma lista simples e calcula a ação do robô.
                # Depois publica a nova velocidade para continuar andando com segurança.
                ranges = list(self.scan.ranges)
                v, w = self.compute_command(ranges)
                self.publish(v, w)
                self.log_data(self.pose, ranges, v, w)

            # Espera o tempo necessário para manter a taxa de 20 Hz.
            rate.sleep()


if __name__ == "__main__":
    # Ponto de entrada do script: instancia a classe e começa a execução do comportamento.
    Avoid().run()
