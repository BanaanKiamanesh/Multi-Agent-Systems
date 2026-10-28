# Dual Preference Agreement

Dual preference agreement lets agents with different preferred states negotiate a shared terminal state. Each agent plans its own scalar integrator trajectory while communication coordinates the final values.

## Mathematical formulation

Agent i minimizes a quadratic sum of state deviation from its preference and input effort, with dynamics $x_i^{k+1}=x_i^k+u_i^k$. Terminal equality is coupled through multipliers. After solving local plans, the dual update has the form

$$\lambda^{\ell+1}=\lambda^\ell+\eta Lx_N^\ell.$$

The terminal Laplacian residual drives disagreement in predicted final states toward zero.

## Collective behavior

Positive local state and input weights make each planning subproblem strictly convex. Preferences determine the compromise, while input penalties account for the effort needed to reach it from current states. Neighbor messages exchange terminal predictions rather than full local trajectories. The multiplier step must be compatible with the dual curvature.

## Assumptions and limitations

The communication graph must be connected and the local cost weights positive. A finite number of dual rounds generally leaves terminal disagreement. The resulting compromise depends on preferences, weights, initial conditions and horizon; it is not necessarily the unweighted mean of preferences. This formulation uses a unit discrete time step and no state or actuator constraints.
