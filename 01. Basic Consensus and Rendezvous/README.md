# 01. Basic Consensus and Rendezvous

Convergence depends on positive-weight connectivity. Directed or row-normalized coupling need not preserve the arithmetic mean.

| Lesson | Example |
|---:|---|
| 01 | [First-Order Undirected Consensus](<01. First-Order Undirected Consensus/README.md>) |
| 02 | [Discrete-Time Consensus](<02. Discrete-Time Consensus/README.md>) |
| 03 | [Metropolis Canal Consensus](<03. Metropolis Canal Consensus/README.md>) |
| 04 | [First-Order Directed Consensus](<04. First-Order Directed Consensus/README.md>) |
| 05 | [Planar Rendezvous](<05. Planar Rendezvous/README.md>) |
| 06 | [Cyclic Pursuit](<06. Cyclic Pursuit/README.md>) |
| 07 | [Max-Min Message Consensus](<07. Max-Min Message Consensus/README.md>) |

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
