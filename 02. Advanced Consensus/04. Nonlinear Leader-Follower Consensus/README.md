# Nonlinear Leader-Follower Consensus

Nonlinear leader–follower consensus makes networked scalar states follow a prescribed reference. Bounded hyperbolic-tangent interactions reduce the magnitude of feedback caused by large disagreements.

## Mathematical formulation

Let r(t) be the leader reference. A follower obeys

$$\dot x_i=\dot r-k\sum_j a_{ij}\tanh(x_i-x_j)-k_p\tanh(x_i-r).$$

The designated leader follows $\dot x_0=\dot r$. Neighbor feedback promotes agreement, pinning attracts agents to the reference and feedforward supplies its known rate.

## Collective behavior

Near agreement, tanh is approximately linear, so the response resembles a pinned linear consensus network. Far from the target, its bounded slope and magnitude soften large corrections. Neighbor influence and reference pinning jointly determine tracking transients.

## Assumptions and limitations

The designated leader must start on its prescribed reference. Reference position and rate are available to the pinned agents. Gains must be positive and graph weights nonnegative for the usual attraction interpretation. This is a model-based tracking law; unknown reference rates, disturbances and actuator limits require separate treatment.
