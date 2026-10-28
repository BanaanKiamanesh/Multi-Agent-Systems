# Olfati-Saber Algorithm 2 2D

Olfati-Saber Algorithm 2 combines local flock formation with a shared navigation reference in planar space. The reference selects collective motion while pair interactions regulate the flock’s internal geometry.

## Mathematical formulation

The acceleration is a sum of spacing, alignment and navigation:

$$\dot v_i=u_i^\alpha-c_p\sigma_1(p_i-r)-c_v(v_i-\dot r).$$

Here $u_i^\alpha$ combines sigma-distance pair potentials with smooth finite-range velocity alignment. The saturation $\sigma_1$ is applied coordinatewise to the position error.

## Collective behavior

Spacing feedback seeks a local lattice-like configuration, alignment suppresses relative velocities, and navigation pulls the group toward the reference. A circular planar reference creates curved collective motion. Reference acceleration is not supplied as feedforward, so accelerating motion can retain tracking error.

## Assumptions and limitations

The desired spacing must be less than the sensing radius, smoothing parameters positive, and force-shape parameters satisfy 0<A≤B. The reference must supply compatible position and velocity. Changing neighborhoods can fragment the flock, and finite interaction forces do not provide a universal collision guarantee. A general exact-tracking claim for accelerating references is therefore inappropriate.
