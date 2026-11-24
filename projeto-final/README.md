# Projeto final — Sumô, rouba-bandeira e futebol

Projeto final de Projeto de Sistemas Robóticos (DCC/UFMG), desenvolvido sobre o kit `psr-hero-kit`.

## Estrutura da pasta

```
projeto-final/
├── sumo.py                  # código oficial do torneio (sumô)
├── ctf.py                   # código oficial do torneio (rouba-bandeira)
├── soccer.py                # código oficial do torneio (futebol)
├── verificar.py             # verificador estático do torneio
├── exercicios/              # versões finais, versões v1 (comparação), logs e vídeos
│   ├── sumo/
│   ├── ctf/
│   └── soccer/
└── ferramentas/             # scripts auxiliares fora do torneio (coleta de logs)
    └── registra_partida.py
```

Os três arquivos na raiz de `projeto-final/` (`sumo.py`, `ctf.py`, `soccer.py`) seguem o formato
padrão lido automaticamente pelo sistema do torneio.

## Verificação antes do envio

Na raiz do repositório:

```bash
python3 projeto-final/verificar.py
```

## Como rodar no kit (`psr-hero-kit`)

Copie os códigos (e as versões `*_v1.py`, para comparação) para dentro de `exercicios/<jogo>/` do kit:

```bash
cp projeto-final/exercicios/sumo/sumo*.py     ~/Work/code/psr-hero-kit/exercicios/sumo/
cp projeto-final/exercicios/ctf/ctf*.py       ~/Work/code/psr-hero-kit/exercicios/ctf/
cp projeto-final/exercicios/soccer/soccer*.py ~/Work/code/psr-hero-kit/exercicios/soccer/
cp projeto-final/ferramentas/registra_partida.py ~/Work/code/psr-hero-kit/exercicios/
```

Dentro da pasta do kit (`./run.sh build` só na primeira vez):

### Sumô
```bash
./run.sh run sumo robot1:=none                       # final (azul) contra parado
./run.sh run sumo robot0:=none robot1:=sumo.py       # final (laranja) contra parado
./run.sh run sumo robot1:=sumo_v1.py                 # final (azul) contra v1 (laranja)
./run.sh run sumo robot0:=sumo_v1.py robot1:=sumo.py # v1 (azul) contra final (laranja)
```

### Rouba-bandeira
```bash
./run.sh run ctf red:=none               # final (azul) contra parado
./run.sh run ctf blue:=none              # final (vermelho) contra parado
./run.sh run ctf red:=ctf_v1.py          # final (azul) contra v1 (vermelho)
./run.sh run ctf blue:=ctf_v1.py         # v1 (azul) contra final (vermelho)
```

### Futebol
```bash
./run.sh run soccer red:=none            # final (azul) contra parado
./run.sh run soccer blue:=none           # final (vermelho) contra parado
./run.sh run soccer red:=soccer_v1.py    # final (azul) contra v1 (vermelho)
./run.sh run soccer blue:=soccer_v1.py   # v1 (azul) contra final (vermelho)
```

## Coleta de registros para o relatório

Com a partida rodando no kit, abra outro terminal na pasta do kit:

```bash
./run.sh shell
python3 /ws/psr_exercicios/registra_partida.py ctf final_vs_v1_azul_1 300
python3 /ws/psr_exercicios/registra_partida.py soccer final_vs_v1_azul_1 300
```

Os arquivos são salvos em `exercicios/<jogo>/logs/<nome>.txt` dentro do kit; depois copie-os para
`projeto-final/exercicios/<jogo>/logs/` no repositório.
