# 03. Formation Control

Displacement, distance and bearing constraints have different translation, rotation and scale freedoms. Nonlinear shape recovery is not globally guaranteed.

| Lesson | Example |
|---:|---|
| 20 | [Third-Order Formation Control](<01. Third-Order Formation Control/README.md>) |
| 21 | [Moving Leader-Follower Formation](<02. Moving Leader-Follower Formation/README.md>) |
| 22 | [Multi-Leader Containment](<03. Multi-Leader Containment/README.md>) |
| 23 | [Distance-Based Formation](<04. Distance-Based Formation/README.md>) |
| 24 | [Henneberg Rigid Formation](<05. Henneberg Rigid Formation/README.md>) |
| 25 | [Bearing Gradient Formation](<06. Bearing Gradient Formation/README.md>) |
| 26 | [Bearing-Only Formation](<07. Bearing-Only Formation/README.md>) |
| 27 | [Pinned Bearing Formation](<08. Pinned Bearing Formation/README.md>) |
| 28 | [Bearing Formation Maneuvering](<09. Bearing Formation Maneuvering/README.md>) |
| 29 | [Second-Order Bearing Maneuvering](<10. Second-Order Bearing Maneuvering/README.md>) |
| 30 | [SE2 Local-Bearing Formation](<11. SE2 Local-Bearing Formation/README.md>) |
| 31 | [Finite-Time Displacement Formation](<12. Finite-Time Displacement Formation/README.md>) |

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
