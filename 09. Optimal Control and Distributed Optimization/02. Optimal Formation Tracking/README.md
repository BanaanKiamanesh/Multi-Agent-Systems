# Optimal Formation Tracking

Optimal formation tracking jointly plans the control inputs of several double-integrator agents. The goal is to follow a moving collective reference while preserving desired inter-agent distances and limiting effort.

## Mathematical formulation

The stage objective combines centroid position and velocity errors, velocity disagreement, control effort and a squared-distance formation term:

$$\frac12\sum_{(i,j)}(\|p_i-p_j\|^2-d_{ij}^2)^2.$$

Exact sampled double-integrator dynamics propagate the trajectory. Backward discrete adjoints calculate the gradient of the complete finite-horizon objective with respect to every control input.

## Collective behavior

A quasi-Newton search updates the joint control trajectory using this gradient. Distance penalties permit global translation and rotation, while the centroid reference and velocity terms select the group maneuver. The terminal cost emphasizes final tracking quality relative to intermediate errors.

## Assumptions and limitations

The nonlinear distance objective is nonconvex, so optimization can converge to a local stationary solution rather than a global optimum. The method is centralized and relies on known agent dynamics and desired geometry. Input and obstacle constraints are absent. Control histories naturally have one fewer sample than state histories because controls act over intervals between states.
