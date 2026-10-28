# Discrete-Time Consensus

Discrete-time consensus replaces continuous attraction with repeated weighted averaging. Every agent updates synchronously from its own state and the most recent neighboring states.

## Mathematical formulation

With Laplacian $L$, the update is

$$x^{\ell+1}=Wx^\ell,\qquad W=I-hL.$$

The sufficient bound $0<h<1/\max_iL_{ii}$ makes all weights nonnegative and leaves a positive self-weight at every node. Each new state is then a convex combination of values available to that agent.

## Collective behavior

A rooted positive-weight communication graph together with the positive diagonal gives agreement. For a connected undirected graph, $W$ is doubly stochastic, preserving the initial arithmetic mean. The magnitudes of the nontrivial eigenvalues of $W$ determine the number of rounds required to reduce disagreement.

## Assumptions and limitations

The step h is an averaging weight, not merely a plotting interval. Excessively large values can produce oscillations or instability. Synchronous updates, a fixed graph and valid communication weights are assumed. Directed networks generally converge to a weighted mean instead of the arithmetic mean.
