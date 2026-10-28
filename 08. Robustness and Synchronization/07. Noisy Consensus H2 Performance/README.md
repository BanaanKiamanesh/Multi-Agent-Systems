# Noisy Consensus H2 Performance

Noisy consensus studies the steady disagreement caused by persistent random disturbances. It characterizes fluctuation energy rather than expecting every trajectory to settle to one fixed state.

## Mathematical formulation

Project noise onto the disagreement subspace with $P_\perp=I-\mathbf1\mathbf1^T/n$. For unit consensus gain, stationary covariance satisfies

$$LC+CL=P_\perp,\qquad C=\frac12L^+.$$

The expected squared disagreement is $\operatorname{tr}(C)$. Exact sampled Ornstein–Uhlenbeck transitions account for both deterministic decay and accumulated noise over each interval.

## Collective behavior

Weakly connected graph modes decay slowly and therefore accumulate more variance. Stronger algebraic connectivity generally reduces disturbance amplification. Projecting off the mean mode avoids an uncontrolled Brownian centroid while retaining the full disagreement process. Ensemble averages approximate expected energy; an individual trajectory continues fluctuating.

## Assumptions and limitations

The covariance formula assumes a connected undirected positive-weight graph and the stated unit isotropic disagreement noise. Different gains or noise covariance require a modified Lyapunov equation. Finite ensembles and observation horizons introduce statistical variation. A covariance prediction describes long-run second moments, not exact pointwise convergence.
