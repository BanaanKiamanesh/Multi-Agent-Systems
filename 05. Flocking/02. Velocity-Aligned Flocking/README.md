# Velocity-Aligned Flocking

Velocity-aligned flocking combines preferred inter-agent spacing, local velocity agreement and navigation toward a common moving reference. The agents seek coherent translation while maintaining neighborhood structure.

## Mathematical formulation

Acceleration contains three parts:

$$\dot v_i=u_i^{\rm spacing}+\sum_j a_{ij}(p)(v_j-v_i)
-k_p(p_i-r)-k_v(v_i-\dot r).$$

Spacing uses a smooth sigma-distance potential. A compactly supported weight turns interaction off beyond the sensing range and smooths its transition near that boundary.

## Collective behavior

The pair potential attracts agents that are too far apart and repels those that are too close relative to the preferred spacing. Alignment damps relative velocities, while the reference terms select collective position and velocity. The sensing graph changes with motion, so local information availability is part of the dynamics.

## Assumptions and limitations

The preferred distance must lie inside the sensing range and smoothing parameters must be positive. Initial connectivity alone does not guarantee connectivity for all time. Coherent motion is a collective objective, not a guarantee of safe separation under every initial condition. Constant reference velocity avoids the need for acceleration feedforward in the nominal maneuver.
