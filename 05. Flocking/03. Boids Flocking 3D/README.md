# Boids Flocking 3D

Boids flocking uses three intuitive local rules: separate from close neighbors, align velocities and move toward the neighborhood center. A goal-following term guides the group through three-dimensional space.

## Mathematical formulation

A representative acceleration is

$$\dot v_i=c_s\sum_{j\in\mathcal N_i}\frac{p_i-p_j}{\|p_i-p_j\|}
+c_a(\bar v_i-v_i)+c_c(\bar p_i-p_i)
+c_g(r-p_i)+c_v(\dot r-v_i).$$

The bars denote neighborhood averages; the averaging neighborhood includes the agent itself. Separation uses unit directions rather than an inverse-square barrier.

## Collective behavior

Separation discourages crowding, cohesion prevents the group from dispersing and alignment promotes common motion. Their gains compete: strong cohesion can overwhelm separation, while weak cohesion can allow fragmentation. A changing waypoint reference adds group-level navigation but can create sharp transients when the desired location switches.

## Assumptions and limitations

This is a behavioral flocking rule rather than a universal stability or safety theorem. Neighbor sets change at the sensing radius, and finite pair forces do not prevent every collision. Coincident positions have no unique separation direction. The model assumes spatial acceleration actuation and does not incorporate aerodynamic or vehicle-specific constraints.
