# Switching-Topology Consensus

Switching-topology consensus changes the communication network while agents are reaching agreement. A sequence of graph Laplacians determines which neighbors exchange information during each interval.

## Mathematical formulation

For a switching signal $\sigma(t)$,

$$\dot x=-kL_{\sigma(t)}x.$$

Each active graph is undirected and connected. The state remains continuous at switches even though the derivative changes. A switching period determines how long each topology remains active.

## Collective behavior

All symmetric Laplacians preserve the same arithmetic mean. The common disagreement energy $V=\frac12\|x-\bar x\mathbf1\|^2$ satisfies $\dot V=-k x^TL_{\sigma(t)}x\le0$. A finite family of connected positive-weight graphs supplies a common positive lower bound on disagreement decay.

## Assumptions and limitations

The shared Lyapunov argument relies on undirected coupling and connected active graphs. It should not be transferred unchanged to arbitrary directed, disconnected or adversarial switching. Numerical steps spanning a switch lose smooth-system accuracy, so switch timing and integration resolution affect the trajectory.
