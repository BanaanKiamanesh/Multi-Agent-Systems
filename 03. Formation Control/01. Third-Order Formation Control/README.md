# Third-Order Formation Control

Third-order displacement formation coordinates positions, velocities and accelerations while preserving a prescribed relative shape. Each agent is assigned a displacement offset, and agreement is applied to position relative to that offset.

## Mathematical formulation

Let $e_i=p_i-d_i$. The jerk law is

$$p^{(3)}=-k_pLe-k_v\dot p-k_a\ddot p.$$

When all offset errors agree, every pair satisfies $p_i-p_j=d_i-d_j$. Absolute velocity and acceleration damping remove higher-order collective motion.

## Collective behavior

The shape has a free translation because adding the same vector to every position does not change Le. A real positive graph eigenvalue produces $s^3+k_as^2+k_vs+k_p\lambda$. Positive coefficients are insufficient: the cubic also requires $k_ak_v>k_p\lambda$. Initial derivative states can shift the final collective translation.

## Assumptions and limitations

The offsets must define consistent geometry and the communication graph must support agreement. Every disagreement mode and the damped collective modes must be stable. Complex directed modes need their actual characteristic roots evaluated. The law assumes independently actuated jerk and does not enforce collision, speed or acceleration constraints.
