# MultiAgent-Control

65 standalone Python implementations of consensus, formation control, coverage,
flocking, multi-agent navigation, localization and distributed optimization.

Each algorithm lives entirely in its own numbered folder. Copy any algorithm
folder elsewhere, install its dependencies, and run it. There is no shared
repository package or dependency on sibling folders.

## Scope

Every example involves interacting agents: graph-coupled states, relative
measurements, shared coverage, formation objectives, inter-agent forces or
visibility constraints. Both distributed protocols and centralized multi-agent
control and estimation are included. Each algorithm README identifies the
interaction and its implementation assumptions.

## Run one algorithm

Use Python 3.10 or newer. For example:

```bash
cd "01. Basic Consensus and Rendezvous/01. First-Order Undirected Consensus"
python -m pip install -r requirements.txt
python main.py
```

A virtual environment is recommended. Each folder contains:

```text
algorithm.py       Controller/planner, numerical helpers and example configuration
plotting.py        Figures and saved results for this algorithm
main.py            Command-line entry point
requirements.txt   External Python dependencies
README.md          Run instructions, theory and assumptions
```

Edit the local `simulate()` function in `algorithm.py` to change initial conditions,
gains or simulation duration. Running `main.py` creates `results/` beside that
script, regardless of the terminal's working directory. Use `--output PATH` for
another destination. Examples with animation support also accept `--animate`, including the 3D flocking examples.

Results include `tracking_plots.svg`, `trajectory.svg` and
`simulation_data.npz`. The arrays include `Time`, `Motion`, `Error` and relevant
numerical metadata. Plots use a noninteractive backend. Generated results are
ignored by Git.

## Explore the examples

The [learning path](LEARNING_PATH.md) lists every folder. The [theory guide](THEORY.md)
collects the equations and assumptions; each algorithm folder includes its own
explanation, assumptions and limitations in its README.

| Topic | Examples |
|---|---:|
| Basic consensus and rendezvous | 7 |
| Advanced consensus | 12 |
| Formation control | 12 |
| Coverage control | 3 |
| Flocking | 9 |
| Multi-agent navigation | 1 |
| Localization and estimation | 8 |
| Robustness and synchronization | 8 |
| Optimal control and distributed optimization | 5 |

These are educational simulations. Convergence, stability and safety statements
depend on graph, gain, model and numerical assumptions documented with each topic.
External MATLAB models and course archives are not required.

## Run all examples

The root dependency list installs the union of the examples' requirements:

```bash
python -m pip install -r requirements.txt
python run_all.py
```

The optional runner starts each algorithm in a separate process. It contains no
algorithm implementations and is not required to run an individual folder.

## References

The following sources support the theory discussed in this repository: consensus
[1], bearing formation and maneuvering [2]–[4], coverage [5], flocking [6],
dynamic average estimation [7], and trajectory optimization [8]. Implementation
assumptions and adaptations are described in [THEORY.md](THEORY.md) and the
individual algorithm READMEs.

[1] R. Olfati-Saber and R. M. Murray, “Consensus problems in networks of agents with switching topology and time-delays,” *IEEE Trans. Autom. Control*, vol. 49, no. 9, pp. 1520–1533, Sep. 2004, doi: [10.1109/TAC.2004.834113](https://doi.org/10.1109/TAC.2004.834113).

[2] S. Zhao and D. Zelazo, “Bearing rigidity and almost global bearing-only formation stabilization,” *IEEE Trans. Autom. Control*, vol. 61, no. 5, pp. 1255–1268, May 2016, doi: [10.1109/TAC.2015.2459191](https://doi.org/10.1109/TAC.2015.2459191).

[3] S. Zhao and D. Zelazo, “Bearing-based formation maneuvering,” in *Proc. IEEE Int. Symp. Intelligent Control (ISIC)*, Sydney, Australia, 2015, pp. 658–663, doi: [10.1109/ISIC.2015.7307285](https://doi.org/10.1109/ISIC.2015.7307285).

[4] S. Zhao and D. Zelazo, “Translational and scaling formation maneuver control via a bearing-based approach,” *IEEE Trans. Control Netw. Syst.*, vol. 4, no. 3, pp. 429–438, Sep. 2017, doi: [10.1109/TCNS.2015.2507547](https://doi.org/10.1109/TCNS.2015.2507547).

[5] J. Cortés, S. Martínez, T. Karatas, and F. Bullo, “Coverage control for mobile sensing networks,” *IEEE Trans. Robot. Autom.*, vol. 20, no. 2, pp. 243–255, Apr. 2004, doi: [10.1109/TRA.2004.824698](https://doi.org/10.1109/TRA.2004.824698).

[6] R. Olfati-Saber, “Flocking for multi-agent dynamic systems: Algorithms and theory,” *IEEE Trans. Autom. Control*, vol. 51, no. 3, pp. 401–420, Mar. 2006, doi: [10.1109/TAC.2005.864190](https://doi.org/10.1109/TAC.2005.864190).

[7] R. A. Freeman, P. Yang, and K. M. Lynch, “Stability and convergence properties of dynamic average consensus estimators,” in *Proc. 45th IEEE Conf. Decision and Control*, San Diego, CA, USA, Dec. 2006, pp. 338–343, doi: [10.1109/CDC.2006.377078](https://doi.org/10.1109/CDC.2006.377078).

[8] J. Hauser, “A projection operator approach to the optimization of trajectory functionals,” *IFAC Proc. Vol.*, vol. 35, no. 1, pp. 377–382, 2002, doi: [10.3182/20020721-6-ES-1901.00312](https://doi.org/10.3182/20020721-6-ES-1901.00312).

## License

[MIT](LICENSE).
