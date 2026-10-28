# Projection Newton Formation

Projection Newton formation optimizes a group trajectory while respecting agent dynamics. The formation objective couples relative positions, centroid motion, velocity error and control effort over a finite horizon.

## Mathematical formulation

For a sampled double integrator,

$$p_{k+1}=p_k+h v_k+\tfrac12h^2u_k,\qquad v_{k+1}=v_k+hu_k.$$

A quadratic cost penalizes deviations from a moving desired formation. A feedback projection maps candidate state–input curves back onto dynamically feasible trajectories, and a backward Riccati recursion determines the linear-quadratic search direction.

## Collective behavior

The projection separates trajectory feasibility from optimization of the cost. An Armijo line search controls the update length. Because the dynamics are linear and the objective quadratic, the subproblem admits an exact linear-quadratic solution; this is a specialization of trajectory projection methods, rather than the full nonlinear optimal-control setting.

## Assumptions and limitations

The communication graph defines an undirected connected formation cost, but the trajectory optimization itself is centralized. Positive input weighting regularizes control effort. The projection tracking feedback must be stable at the chosen sample period. No obstacle or actuator constraints are included, so an optimal cost does not certify a physically feasible constrained maneuver.
