#!/usr/bin/env python3
"""Grava o placar de uma partida a partir de /game/state.

Uso, com o jogo rodando (em outro terminal):
  ./run.sh shell
  python3 /ws/psr_exercicios/registra_partida.py <nome> [duracao_s]

Salva exercicios/registros/<nome>.txt com cada mudança de placar e o resultado
da partida (contado a partir do momento em que o script começou).
"""
import json
import os
import sys

import rospy
from std_msgs.msg import String

KEYS = ("score", "tags", "blue", "red", "wins", "draws")


class Registro:
    def __init__(self, nome, duracao):
        self.duracao = duracao
        self.t0 = None
        self.inicio = None
        self.ultimo = None
        pasta = os.path.join(os.path.dirname(os.path.abspath(__file__)), "registros")
        os.makedirs(pasta, exist_ok=True)
        caminho = os.path.join(pasta, nome + ".txt")
        self.arq = open(caminho, "w")
        # o container roda como root: deixa o arquivo editável fora dele
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
    if len(args) < 2:
        print("uso: registra_partida.py <nome> [duracao_s]")
        sys.exit(1)
    rospy.init_node("registra_partida", anonymous=True)
    Registro(args[1], float(args[2]) if len(args) > 2 else 300.0)
    rospy.spin()
