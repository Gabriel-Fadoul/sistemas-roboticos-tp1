# Sumô

Códigos do sumô do projeto final.

- `sumo.py`: versão final
- `sumo_v1.py`: primeira versão, usada para comparar com a final
- `logs/`: saídas do terminal dos testes que usamos no relatório
- vídeo do sumô: [colocar o nome do arquivo]

A única diferença entre as duas versões é quando o robô começa a empurrar. Na v1 ele empurra assim que chega a 0,16 m do adversário. Na final ele espera chegar perto do ponto de ataque, que fica ao lado e atrás do adversário (ou passar 1,5 s).

## Como rodar

Precisa do kit da disciplina (`psr-hero-kit`) já instalado. No Windows, rodamos tudo pelo Ubuntu no WSL.

Primeiro copie os dois arquivos para a pasta do sumô do kit e entre na pasta do kit:

```bash
cp sumo.py sumo_v1.py ~/psr-hero-kit/exercicios/sumo/
cd ~/psr-hero-kit
```

Depois é só escolher o teste. `robot0` é o robô azul e `robot1` é o laranja. `none` deixa o robô parado.

```bash
# final contra robô parado
./run.sh run sumo robot1:=none                    # final no azul
./run.sh run sumo robot0:=none robot1:=sumo.py    # final no laranja

# final contra a v1
./run.sh run sumo robot1:=sumo_v1.py                    # final no azul
./run.sh run sumo robot0:=sumo_v1.py robot1:=sumo.py    # final no laranja

# v1 contra ela mesma
./run.sh run sumo robot0:=sumo_v1.py robot1:=sumo_v1.py
```

Para parar é `Ctrl+C`.

No terminal aparecem as trocas de estado do robô (`approach`, `push`, `recover`) e o placar de cada rodada, por exemplo `PLACAR hero_0: wins=[6, 3] draws=11`. O primeiro número de `wins` é do azul e o segundo do laranja.

## Logs

- `log_final_vs_parado_azul.txt` e `log_final_vs_parado_laranja.txt`: final contra o robô parado. Ganhou todas dos dois lados.
- `log_final_vs_v1_azul.txt`: final no azul contra a v1. 6 vitórias, 3 derrotas e 11 empates.
- `log_final_vs_v1_laranja.txt`: final no laranja contra a v1. 7 vitórias, 6 derrotas e 15 empates.
- `log_v1_vs_v1.txt`: v1 contra ela mesma. Azul 4, laranja 11 e 5 empates.
