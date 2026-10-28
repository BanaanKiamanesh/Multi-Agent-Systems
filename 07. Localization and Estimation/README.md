# 07. Localization and Estimation

Identifiability depends on the graph and anchors. Range reconstruction is nonconvex; changing-signal estimation and finite gossip rounds generally retain residual error.

| Lesson | Example |
|---:|---|
| 45 | [Relative Measurement Localization](<01. Relative Measurement Localization/README.md>) |
| 46 | [Proximal Relative Estimation](<02. Proximal Relative Estimation/README.md>) |
| 47 | [Range-Only Frame Estimation](<03. Range-Only Frame Estimation/README.md>) |
| 48 | [Gauss-Newton Range Localization](<04. Gauss-Newton Range Localization/README.md>) |
| 49 | [Bearing-Based Localization](<05. Bearing-Based Localization/README.md>) |
| 50 | [Dynamic Average Estimation](<06. Dynamic Average Estimation/README.md>) |
| 51 | [Local Network State Observer](<07. Local Network State Observer/README.md>) |
| 52 | [Distributed Rigidity Eigenvector](<08. Distributed Rigidity Eigenvector/README.md>) |

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
