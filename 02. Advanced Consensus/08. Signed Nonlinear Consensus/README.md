# Signed Nonlinear Consensus

Signed nonlinear consensus allows attractive and repulsive edges in a scalar network. Positive weights encourage equal states, whereas negative weights favor separation.

## Mathematical formulation

For an edge-oriented incidence matrix B and signed weights $w_e$,

$$\dot x=-kB\operatorname{diag}(w)\tanh(B^Tx).$$

This is the gradient flow of $U(x)=k\sum_e w_e\log\cosh((B^Tx)_e)$. Each edge applies equal and opposite forces to its endpoints.

## Collective behavior

Pairwise antisymmetry preserves the arithmetic mean. Along the ideal flow, the potential cannot increase. However, negative weights destroy convexity and may make the potential unbounded below, so decreasing potential alone does not establish boundedness or agreement. The signed linearization near consensus reveals whether small disagreements grow.

## Assumptions and limitations

The signs and magnitudes of repulsive edges must be assessed relative to the attractive network. This is not the structurally balanced bipartite protocol that compares signed endpoint values. Opposite-sign clusters and global convergence cannot be inferred merely from the presence of negative weights. Smooth bounded forces can still produce unbounded separation over time.
