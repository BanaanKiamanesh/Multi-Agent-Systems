# Finite-Time Consensus

Finite-time consensus uses a nonlinear attraction that remains strong relative to small disagreement. Unlike ordinary linear agreement, its ideal continuous-time dynamics can reach exact agreement in finite time.

## Mathematical formulation

For $0<\alpha<1$,

$$\dot x_i=k\sum_j a_{ij}\operatorname{sign}(x_j-x_i)|x_j-x_i|^\alpha.$$

The edge interaction is antisymmetric on an undirected graph, preserving the arithmetic mean. The exponent controls the nonlinear response near zero.

## Collective behavior

A disagreement Lyapunov function admits a decay inequality of the form $\dot V\le-cV^\beta$ with $0<\beta<1$. Integrating that inequality yields a finite settling-time bound. Smaller exponents strengthen the correction relative to the remaining error near agreement, but also make the vector field less smooth.

## Assumptions and limitations

The finite-time statement assumes a connected undirected graph, positive edge weights, positive gain and an exponent strictly between zero and one. Fixed-step numerical trajectories generally exhibit a small residual or chatter rather than exact finite-time termination. Large time steps are especially problematic near the nonsmooth consensus set.
