# 09. Optimal Control and Distributed Optimization

The LQ projection problem is convex; nonlinear distance objectives admit local solutions. Finite communication budgets do not enforce exact terminal agreement.

| Lesson | Example |
|---:|---|
| 61 | [Projection Newton Formation](<01. Projection Newton Formation/README.md>) |
| 62 | [Optimal Formation Tracking](<02. Optimal Formation Tracking/README.md>) |
| 63 | [Dual Preference Agreement](<03. Dual Preference Agreement/README.md>) |
| 64 | [Shrinking-Horizon Preference Agreement](<04. Shrinking-Horizon Preference Agreement/README.md>) |
| 65 | [Online Gradient Formation Tracking](<05. Online Gradient Formation Tracking/README.md>) |

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
