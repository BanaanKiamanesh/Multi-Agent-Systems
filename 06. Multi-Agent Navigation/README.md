# 06. Multi-Agent Navigation

Robots navigate toward individual goals while reacting to other robots and spherical obstacles. Bounded potential forces and sampled integration do not guarantee collision avoidance.

| Lesson | Example |
|---:|---|
| 44 | [3D Multi-Agent Obstacle Avoidance](<01. 3D Multi-Agent Obstacle Avoidance/README.md>) |

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
