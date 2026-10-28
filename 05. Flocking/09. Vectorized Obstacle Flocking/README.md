# Vectorized Obstacle Flocking

Obstacle flocking guides a group through the plane toward a goal while reacting to nearby agents and obstacles. It combines local alignment, repulsive interactions and tangential obstacle motion.

## Mathematical formulation

Acceleration combines

$$\dot v_i=u_i^{\rm pair}+u_i^{\rm align}+u_i^{\rm obstacle}
+k_g\frac{r-p_i}{\|r-p_i\|+\varepsilon}-cv_i.$$

The pair force uses a shifted finite-range repulsive shaping function. Obstacle response combines exponential normal repulsion with a fixed rotated planar tangent. The agents are also subject to a speed cap and bounded arena.

## Collective behavior

Goal attraction moves the group toward the destination, while alignment promotes coordinated velocities. Tangential motion helps pass an obstacle rather than approach its normal directly. Arrival is a collective stopping criterion based on a required fraction of agents inside a goal region; it does not mean every robot has arrived. The position update includes an acceleration term and additional damping, yielding a velocity-dependent Verlet-style scheme rather than conservative Verlet integration.

## Assumptions and limitations

This practical obstacle variant differs from the published projected beta-agent controller. Finite repulsive forces, speed caps and boundary clipping prevent a universal safety or convergence claim. Coincident agent or obstacle-center positions make geometric directions ambiguous. Neighbor connectivity can change, and a majority-arrival criterion can leave lagging agents outside the goal region.
