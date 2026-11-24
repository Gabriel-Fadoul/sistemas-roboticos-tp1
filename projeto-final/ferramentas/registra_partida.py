#!/usr/bin/env python3
"""Ferramenta auxiliar (NAO faz parte do torneio) para gravar logs das partidas de 5 min.

Uso, com o jogo rodando no kit (em outro terminal):
  ./run.sh shell
  python3 /ws/psr_exercicios/registra_partida.py <jogo> <nome_log> [duracao_s]

Exemplos:
  python3 /ws/psr_exercicios/registra_partida.py ctf final_vs_parado_azul_1 300
  python3 /ws/psr_exercicios/registra_partida.py soccer final_vs_v1_azul_1 300

Salva em exercicios/<jogo>/logs/<nome_log>.txt cada mudanca de placar e o resumo final.
"""
import json
import os
import sys

import rospy
from std_msgs.msg import String

KEYS = ("score", "tags", "blue", "red", "wins", "draws")


class Registro:
    def __init__(self, jogo, nome, duracao):
        self.duracao = duracao
        self.t0 = None
        self.inicio = None
        self.ultimo = None
        base = os.path.dirname(os.path.abspath(__file__))
        pasta = os.path.join(base, jogo, "logs")
        os.makedirs(pasta, exist_ok=True)
        caminho = os.path.join(pasta, nome + ".txt")
        self.arq = open(caminho, "w")
        os.chmod(pasta, 0o777)
        os.chmod(caminho, 0o666)
        rospy.Subscriber("/game/state", String, self.on_state, queue_size=10)

    def escreve(self, linha):
        print(linha)
        self.arq.write(linha + "\n")
        self.arq.flush()

    def on_state(self, msg):
        if self.arq.closed:
            return
        g = json.loads(msg.data)
        placar = {k: g[k] for k in KEYS if k in g}
        if self.t0 is None:
            self.t0, self.inicio = g["t"], placar
            self.escreve("inicio t=%.1f %s" % (g["t"], json.dumps(placar)))
        t = g["t"] - self.t0
        if placar != self.ultimo:
            self.ultimo = placar
            self.escreve("%6.1f s  %s  %s" % (t, json.dumps(placar), g.get("message", "")))
        if t >= self.duracao:
            final = {}
            for k, v in placar.items():
                v0 = self.inicio[k]
                final[k] = [a - b for a, b in zip(v, v0)] if isinstance(v, list) else v - v0
            self.escreve("fim %.1f s  resultado da partida: %s" % (t, json.dumps(final)))
            self.arq.close()
            rospy.signal_shutdown("fim da partida")


if __name__ == "__main__":
    args = rospy.myargv(argv=sys.argv)
    if len(args) < 3:
        print("uso: registra_partida.py <sumo|ctf|soccer> <nome_log> [duracao_s]")
        sys.exit(1)
    rospy.init_node("registra_partida", anonymous=True)
    Registro(args[1], args[2], float(args[3]) if len(args) > 3 else 300.0)
    rospy.spin()
