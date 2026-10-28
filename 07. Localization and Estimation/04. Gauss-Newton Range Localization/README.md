# Gauss-Newton Range Localization

Gauss–Newton range localization uses a local linearization of distance residuals to accelerate anchored geometric reconstruction. It computes a damped correction rather than following only the raw gradient.

## Mathematical formulation

With squared-range residual r and Jacobian R, a correction satisfies

$$(R_f^TR_f+\mu I)\Delta p_f=-R_f^Tr,\qquad \mu>0.$$

Only free coordinates are solved; anchors remain fixed. Relaxed block-Jacobi iterations approximately solve these normal equations, followed by a coordinated backtracking line search.

## Collective behavior

The approximate Hessian captures how nearby distance constraints interact and often gives better-scaled corrections than steepest descent. Damping regularizes weakly observable directions. The line search accepts a reduced step when a full correction fails to decrease the nonlinear residual objective.

## Assumptions and limitations

The range objective remains nonconvex, so Gauss–Newton does not guarantee a global realization. Identifiable anchored geometry and a suitable initial estimate matter. Finite inner iterations produce approximate directions. The global residual evaluation and coordinated line search mean the overall method is not fully decentralized, even though its linear solve uses local blocks.
