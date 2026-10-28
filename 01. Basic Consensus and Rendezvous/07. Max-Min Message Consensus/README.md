# Max-Min Message Consensus

Max–min message consensus lets every agent discover the largest and smallest initial values in a network. It propagates extrema rather than averaging, so the final pair retains information about the full initial range.

## Mathematical formulation

Initialize $M_i^0=m_i^0=x_i^0$. At each synchronous communication round,

$$M_i^{\ell+1}=\max_{j\in\mathcal N_i\cup\{i\}}M_j^\ell,\qquad
m_i^{\ell+1}=\min_{j\in\mathcal N_i\cup\{i\}}m_j^\ell.$$

Including the current agent prevents previously learned extrema from being lost.

## Collective behavior

After r rounds, an agent knows the extrema among nodes at most r hops away. On a connected undirected graph, the graph diameter therefore bounds the rounds needed for every node to know the global maximum and minimum. Maximum estimates never decrease; minimum estimates never increase.

## Assumptions and limitations

The diameter bound assumes synchronous, reliable communication and a fixed connected graph. A disconnected graph yields componentwise extrema. This discrete finite-round information propagation is distinct from finite-time convergence of a continuous differential equation.
