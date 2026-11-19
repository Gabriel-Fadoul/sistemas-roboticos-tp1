# Projeto final — Sumô, rouba-bandeira e futebol

Código do projeto final de Projeto de Sistemas Robóticos (DCC/UFMG), feito sobre o kit
`psr-hero-kit`.

## Como rodar

Copie o conteúdo de `exercicios/` deste repositório para a pasta `exercicios/` do kit
(substituindo os modelos) e, dentro da pasta do kit:

```bash
./run.sh build                           # só na primeira vez
```

### Rouba-bandeira (`exercicios/ctf/`)

| Arquivo | O que é |
|---|---|
| `ctf.py` | versão final |
| `ctf_v1.py` | primeira versão, usada nas comparações |

```bash
./run.sh run ctf red:=none               # nosso time (azul) contra um time parado
./run.sh run ctf blue:=none              # nosso time (vermelho) contra um time parado
./run.sh run ctf red:=ctf_v1.py          # final (azul) contra a v1 (vermelho)
./run.sh run ctf blue:=ctf_v1.py         # v1 (azul) contra a final (vermelho)
```

## Registros das partidas

Com um jogo rodando, em outro terminal:

```bash
./run.sh shell
python3 /ws/psr_exercicios/registra_partida.py ctf_final_x_v1_azul 300
```

O script grava em `exercicios/registros/<nome>.txt` cada mudança de placar e, depois de
300 s de simulação, o resultado da partida.
