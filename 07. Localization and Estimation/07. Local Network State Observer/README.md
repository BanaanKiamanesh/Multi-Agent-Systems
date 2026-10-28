# Local Network State Observer

A local network-state observer gives every agent an estimate of every agent’s signal. Each observer shares its estimate bank with neighbors and corrects the entry corresponding to its own directly measured signal.

## Mathematical formulation

If $\hat s_{ij}$ is observer i’s estimate of signal j, its dynamics combine

$$\dot{\hat s}_{ij}=-k\sum_\ell a_{i\ell}(\hat s_{ij}-\hat s_{\ell j})
-k\delta_{ij}(\hat s_{ii}-s_i)+\dot s_j.$$

The last term represents supplied drift information for the modeled moving signals.

## Collective behavior

For each target signal, its owning agent acts as a pinned information source. Neighbor consensus spreads that correction to the rest of the observer network. With compatible drift feedforward, the collective observer can follow translated moving formations rather than repeatedly lagging behind static estimates.

## Assumptions and limitations

Each signal source must be reachable through the observer graph. Exact tracking of moving signals requires the stated common drift information; it is not a consequence of neighbor averaging alone. Every observer stores a full network estimate, so storage grows with the number of agents. Communication and measurement frames must be consistent.
