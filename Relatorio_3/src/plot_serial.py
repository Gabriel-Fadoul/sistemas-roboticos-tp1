#!/usr/bin/env python3
"""Plota ao vivo o sinal e a distância enviados pela Mega.

Aceita uma coluna (sinal) ou duas colunas (sinal e distância),
separadas por espaço ou vírgula.

    python3 plot_serial.py /dev/ttyACM0
"""

import sys
import time
from collections import deque
import serial

import matplotlib
matplotlib.use("QtAgg")
import matplotlib.pyplot as plt

porta = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
ser = serial.Serial(porta, 9600, timeout=1)
time.sleep(2)  # a Mega reinicia ao abrir a USB



n = 0
xs = deque(maxlen=200)
sinais = deque(maxlen=200)
distancias = deque(maxlen=200)
modo = None

plt.ion()
fig, (ax_sinal, ax_dist) = plt.subplots(2, 1, sharex=True)
(linha_sinal,) = ax_sinal.plot([], [], color="#960B22", lw=1.5)
(linha_dist,) = ax_dist.plot([], [], color="#176B5B", lw=1.5)
ax_sinal.set_ylabel("sinal (contagens)")
ax_dist.set_ylabel("distância (mm)")
ax_dist.set_xlabel("amostra")
fig.tight_layout()


def amostra(bruto):
    partes = bruto.replace(",", " ").split()
    if len(partes) not in (1, 2):
        return None
    try:
        valores = tuple(float(parte) for parte in partes)
    except ValueError:
        return None
    return valores


try:
    while True:
        bruto = ser.readline().decode(errors="ignore").strip()
        lido = amostra(bruto)
        if lido is None:
            continue
        if modo is None:
            modo = len(lido)
            if modo == 1:
                ax_dist.set_visible(False)
        if len(lido) != modo:
            continue
        n += 1
        xs.append(n)
        sinais.append(lido[0])
        if modo == 2:
            distancias.append(lido[1])
        print(bruto, flush=True)
        linha_sinal.set_data(xs, sinais)
        ax_sinal.relim()
        ax_sinal.autoscale_view()
        if modo == 2:
            linha_dist.set_data(xs, distancias)
            ax_dist.relim()
            ax_dist.autoscale_view()
        plt.pause(0.01)
except KeyboardInterrupt:
    pass
finally:
    ser.close()
