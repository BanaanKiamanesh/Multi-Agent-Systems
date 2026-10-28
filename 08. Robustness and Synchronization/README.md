# 08. Robustness and Synchronization

Each method has its own graph and model assumptions. Barrier arguments are continuous-time statements; numerical steps can cross a boundary.

| Lesson | Example |
|---:|---|
| 53 | [Connectivity-Preserving Formation](<01. Connectivity-Preserving Formation/README.md>) |
| 54 | [Rigidity Maintenance](<02. Rigidity Maintenance/README.md>) |
| 55 | [Effective Resistance Robustness](<03. Effective Resistance Robustness/README.md>) |
| 56 | [Objective-Coded Robust Consensus](<04. Objective-Coded Robust Consensus/README.md>) |
| 57 | [Event-Triggered Consensus](<05. Event-Triggered Consensus/README.md>) |
| 58 | [Riccati Linear Synchronization](<06. Riccati Linear Synchronization/README.md>) |
| 59 | [Noisy Consensus H2 Performance](<07. Noisy Consensus H2 Performance/README.md>) |
| 60 | [Field-of-View Relay Control](<08. Field-of-View Relay Control/README.md>) |

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
