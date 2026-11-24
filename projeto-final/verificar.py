#!/usr/bin/env python3
"""Check the three tournament files with the same static rules the tournament collector uses.

  python3 projeto-final/verificar.py

Run it from the root of the repository (or from inside projeto-final/). It does not run the code:
test the behaviour in the kit (./run.sh run sumo robot1:=none and so on).
"""

import ast
import sys
from pathlib import Path

GAMES = ("sumo", "ctf", "soccer")
ALLOWED_TOP = {"numpy", "rospy", "psr_hero", "geometry_msgs", "nav_msgs", "sensor_msgs", "std_msgs"}
STDLIB = set(getattr(sys, "stdlib_module_names", ()))
FORBIDDEN = {"subprocess", "socket", "multiprocessing", "ctypes"}


def check(path: Path) -> list:
    problems = []
    src = path.read_text(encoding="utf-8")
    if not src.strip():
        return ["arquivo vazio"]
    if not src.startswith("#!/usr/bin/env python3"):
        problems.append("a primeira linha deve ser #!/usr/bin/env python3")
    try:
        tree = ast.parse(src)
    except SyntaxError as e:
        return problems + [f"erro de sintaxe na linha {e.lineno}: {e.msg}"]
    for node in ast.walk(tree):
        names = []
        if isinstance(node, ast.Import):
            names = [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                problems.append("import relativo: o arquivo não pode depender de outros arquivos")
                continue
            names = [node.module or ""]
        for name in names:
            top = name.split(".")[0]
            if top in FORBIDDEN:
                problems.append(f"import não permitido: {name}")
            elif top == "psr_hero" and name not in ("psr_hero", "psr_hero.utils"):
                problems.append(f"import não permitido: {name} (use só psr_hero.utils)")
            elif top not in ALLOWED_TOP and STDLIB and top not in STDLIB:
                problems.append(f"import não permitido: {name}")
        if isinstance(node, ast.Call) and getattr(node.func, "id", getattr(node.func, "attr", "")) == "open":
            problems.append(f"linha {node.lineno}: open() — a pasta é somente leitura no torneio; grave só em /tmp")
    if "TODO" in src:
        problems.append("ainda há TODO no arquivo (aviso)")
    return problems


def main() -> None:
    here = Path.cwd()
    folder = here / "projeto-final" if (here / "projeto-final").is_dir() else here
    ok = True
    for g in GAMES:
        path = folder / f"{g}.py"
        if not path.exists():
            print(f"✗ {g}.py: não encontrado em {folder}")
            ok = False
            continue
        problems = check(path)
        warnings = [p for p in problems if p.endswith("(aviso)") or p.startswith("linha")]
        errors = [p for p in problems if p not in warnings]
        ok = ok and not errors
        print(f"{'✗' if errors else '!' if warnings else '✓'} {g}.py")
        for p in errors + warnings:
            print(f"    - {p}")
    print("\nTudo certo para o torneio." if ok else "\nCorrija os itens marcados com ✗.")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
