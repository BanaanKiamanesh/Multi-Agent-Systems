# Second-Order Consensus

Second-order consensus coordinates agents whose positions evolve through velocities. Position disagreement creates acceleration, while absolute velocity damping removes collective drift.

## Mathematical formulation

Writing q for position and v for velocity,

$$\dot q=v,\qquad \dot v=-g_0Lq-g_1v.$$

Each nonzero Laplacian eigenvalue $\lambda$ produces the modal polynomial $s^2+g_1s+g_0\lambda$. The normalized graph matrix may be used to scale incoming influence by node degree.

## Collective behavior

For a connected undirected positive-weight graph, ordinary and row-normalized Laplacians have real positive disagreement eigenvalues. Positive gains stabilize these modes. Positions agree and velocities decay to zero. Initial collective velocity can shift the final common position before damping removes that motion.

## Assumptions and limitations

The zero position mode remains free, but its velocity mode must be damped. Complex directed eigenvalues require checking the actual modal roots; positive coefficients alone are not a universal directed-graph certificate. Row normalization generally gives a degree-weighted rather than arithmetic position invariant. Numerical stability also depends on the integration step.
