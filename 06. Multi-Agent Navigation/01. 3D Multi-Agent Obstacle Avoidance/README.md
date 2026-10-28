# 3D Multi-Agent Obstacle Avoidance

Multi-agent obstacle avoidance guides robots toward individual spatial goals while accounting for both static obstacles and the other robots. Pairwise repulsion makes their trajectories genuinely coupled.

## Mathematical formulation

Each double-integrator robot obeys

$$\dot p_i=v_i,\qquad
\dot v_i=\operatorname{clip}\big(k_g(g_i-p_i)-cv_i+f_i^{\rm obs}+f_i^{\rm pair}\big).$$

Obstacle clearance subtracts obstacle and robot radii from center distance. Pair clearance subtracts two robot radii. Repulsion activates within a safety distance and grows as clearance decreases.

## Collective behavior

Goal attraction supplies the nominal route, damping limits oscillation, and radial repulsion diverts trajectories from occupied regions. Every robot sees a moving environment because neighboring robots also move. Acceleration clipping reflects a bounded control action and changes the balance between attraction and repulsion when commands saturate.

## Assumptions and limitations

Obstacles are spheres and robots are represented by a common radius. The softened inverse-clearance force remains finite, so it is not an impenetrable barrier. Coincident centers have no well-defined normal. The law can stall at potential-field equilibria and has no general collision-free or goal-reaching guarantee, especially under saturation or coarse temporal resolution.
