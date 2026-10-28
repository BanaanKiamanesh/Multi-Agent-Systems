# Fractional-Order Consensus

Fractional-order consensus introduces memory into the evolution of network disagreement. The fractional derivative blends past state changes rather than depending only on the instantaneous rate.

## Mathematical formulation

The network follows the fractional relation

$$D^\alpha x=-k\tanh(Lx),\qquad 0<\alpha\le1.$$

The hyperbolic tangent acts componentwise on Laplacian disagreement. Initialized Grünwald–Letnikov memory is applied to $x-x(0)$, separating the initial condition from the memory sum.

## Collective behavior

The order alpha changes the temporal response and memory strength. At order one the model approaches an ordinary differential equation, while smaller orders often produce slower memory-dominated relaxation. The tanh of a Laplacian sum differs from a sum of pairwise tanh interactions, so pairwise cancellation cannot be assumed.

## Assumptions and limitations

In particular, the arithmetic mean is not generally preserved. Graph coupling, gain and fractional order must be considered together when assessing agreement. Explicit discrete forcing introduces step-size restrictions. A full-history fractional approximation has increasing memory and computational cost as the time horizon is refined; integer-order consensus theorems do not automatically establish fractional nonlinear stability.
