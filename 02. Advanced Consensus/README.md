# 02. Advanced Consensus

Graph, gain, delay and initial-state assumptions matter. Finite-time statements describe continuous dynamics, not exact termination of a fixed-step simulation.

| Lesson | Example |
|---:|---|
| 08 | [Second-Order Consensus](<01. Second-Order Consensus/README.md>) |
| 09 | [Linear State-Space Consensus](<02. Linear State-Space Consensus/README.md>) |
| 10 | [Fourth-Order Consensus](<03. Fourth-Order Consensus/README.md>) |
| 11 | [Nonlinear Leader-Follower Consensus](<04. Nonlinear Leader-Follower Consensus/README.md>) |
| 12 | [Switching-Topology Consensus](<05. Switching-Topology Consensus/README.md>) |
| 13 | [Finite-Time Consensus](<06. Finite-Time Consensus/README.md>) |
| 14 | [Power-Mean Consensus](<07. Power-Mean Consensus/README.md>) |
| 15 | [Signed Nonlinear Consensus](<08. Signed Nonlinear Consensus/README.md>) |
| 16 | [Kuramoto Synchronization](<09. Kuramoto Synchronization/README.md>) |
| 17 | [Delayed Consensus](<10. Delayed Consensus/README.md>) |
| 18 | [Fractional-Order Consensus](<11. Fractional-Order Consensus/README.md>) |
| 19 | [Heterogeneous Fractional Network](<12. Heterogeneous Fractional Network/README.md>) |

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
