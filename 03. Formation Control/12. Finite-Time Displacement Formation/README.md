# Finite-Time Displacement Formation

Finite-time displacement formation reaches a prescribed pattern by applying finite-time consensus to offset-corrected positions. Desired offsets specify both scale and orientation, leaving a free translation.

## Mathematical formulation

For $e_i=p_i-d_i$ and $0<\alpha<1$,

$$\dot p_i=k\sum_j a_{ij}\operatorname{sign}(e_j-e_i)|e_j-e_i|^\alpha,$$

where sign and power act componentwise. Edge contributions are antisymmetric. Agreement of all offset errors gives the desired relative displacements.

## Collective behavior

The nonsmooth power law produces stronger feedback relative to a small remaining error than a linear law. On a connected undirected positive-weight graph, continuous-time finite-time consensus arguments apply separately to each coordinate. The common offset error is determined by its initial average, locating the translated formation.

## Assumptions and limitations

Target displacements must be consistent, the exponent must lie strictly between zero and one, and the gain must be positive. Exact finite-time convergence concerns the continuous dynamics; discrete integration can leave a residual or oscillation. Formation agreement does not provide collision protection during the transient.
