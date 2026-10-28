# 05. Flocking

Local flocking does not guarantee persistent connectivity or collision avoidance. The obstacle Algorithm 3 examples are practical archive variants, not the published beta-agent controller.

| Lesson | Example |
|---:|---|
| 35 | [Swarming and Collective Motion](<01. Swarming and Collective Motion/README.md>) |
| 36 | [Velocity-Aligned Flocking](<02. Velocity-Aligned Flocking/README.md>) |
| 37 | [Boids Flocking 3D](<03. Boids Flocking 3D/README.md>) |
| 38 | [Olfati-Saber Algorithm 1](<04. Olfati-Saber Algorithm 1/README.md>) |
| 39 | [Olfati-Saber Algorithm 2 2D](<05. Olfati-Saber Algorithm 2 2D/README.md>) |
| 40 | [Olfati-Saber Algorithm 2 3D](<06. Olfati-Saber Algorithm 2 3D/README.md>) |
| 41 | [Olfati-Saber Algorithm 3 2D](<07. Olfati-Saber Algorithm 3 2D/README.md>) |
| 42 | [Olfati-Saber Algorithm 3 3D](<08. Olfati-Saber Algorithm 3 3D/README.md>) |
| 43 | [Vectorized Obstacle Flocking](<09. Vectorized Obstacle Flocking/README.md>) |

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
