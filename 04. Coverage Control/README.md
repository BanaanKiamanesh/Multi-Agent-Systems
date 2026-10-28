# 04. Coverage Control

Voronoi cells are approximated by a sampling grid. Moving density defines a changing objective, not a fixed cost with guaranteed monotone descent.

| Lesson | Example |
|---:|---|
| 32 | [Centroidal Voronoi Coverage](<01. Centroidal Voronoi Coverage/README.md>) |
| 33 | [Three-Dimensional Coverage](<02. Three-Dimensional Coverage/README.md>) |
| 34 | [Adaptive Coverage Control](<03. Adaptive Coverage Control/README.md>) |

[Theory and assumptions](../THEORY.md) · [Full learning path](../LEARNING_PATH.md)

## Run an algorithm independently

Each linked child folder is a complete implementation with its own algorithm,
assumptions, dependency list and command-line entry point. Copy the child folder
anywhere, open a terminal there, and run:

```bash
python -m venv .venv
# Linux/macOS:
source .venv/bin/activate
# Windows PowerShell: .venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python main.py
```

Use Python 3.10 or newer. See the child README for equations, limitations and
animation support. Outputs are written to that child folder’s `results/` directory;
no parent or sibling code is needed.
