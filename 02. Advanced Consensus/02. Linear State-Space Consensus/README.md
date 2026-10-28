# Linear State-Space Consensus

Linear state-space consensus coordinates position and velocity through a local linear plant. It combines relative position feedback, relative velocity feedback and absolute damping.

## Mathematical formulation

For a double-integrator state $z_i=[q_i,v_i]^T$,

$$\dot z_i=Az_i+Bu_i,\quad
u_i=-k_p(Lq)_i-k_n(Lv)_i-k_vv_i.$$

The three terms correct positional disagreement, suppress differences in velocity and dissipate collective velocity, respectively.

## Collective behavior

A disagreement eigenvalue $\lambda$ gives $s^2+(k_v+k_n\lambda)s+k_p\lambda$. Real positive graph modes are stable for positive position and absolute-damping gains with nonnegative neighbor damping. The zero mode has a free position and a damped velocity, so agents settle at a common position.

## Assumptions and limitations

The agents share the same plant and communication matrix. Stability for a directed graph with complex eigenvalues must be established mode by mode. Agreement concerns the complete state, not just position. Row-normalized coupling changes the conserved weighted combination and finite integration steps impose additional constraints.
