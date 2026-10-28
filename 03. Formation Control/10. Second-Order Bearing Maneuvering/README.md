# Second-Order Bearing Maneuvering

Second-order bearing maneuvering coordinates accelerating followers around leaders moving at compatible constant velocities. Position and velocity coupling preserve a common bearing pattern during translation and scaling.

## Mathematical formulation

Using the desired-bearing Laplacian B,

$$\dot p=v,\qquad \dot v_f=-\big[B(k_pp+k_vv)\big]_f.$$

Leader accelerations are zero. A compatible motion has $p^*(t)=s(t)p^0+c(t)$ with constant scale rate and translational velocity.

## Collective behavior

The position term removes bearing-line error and the velocity term damps relative motion outside the permitted maneuver. For a positive-definite follower block, each eigenmode gives a damped second-order tracking equation. Leaders fix the translation and scale freedoms that would remain without pinning.

## Assumptions and limitations

Leader initial positions and velocities must be compatible with the target bearings. Positive gains and a positive-definite anchored block provide the real-mode stability conditions. Two leaders by themselves do not guarantee an identifiable formation. Accelerating leaders require additional analysis or feedforward, and vanishing or negative target scale invalidates the intended geometry.
