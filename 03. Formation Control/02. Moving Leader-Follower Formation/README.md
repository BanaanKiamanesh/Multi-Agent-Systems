# Moving Leader-Follower Formation

Moving leader–follower formation maintains relative offsets while the group tracks a moving reference. Agents combine relative shape correction with velocity damping and reference feedforward.

## Mathematical formulation

With $e_i=p_i-d_i-r(t)$, follower acceleration has the form

$$\ddot p_i=\ddot r-k\sum_j a_{ij}(e_i-e_j)-c(\dot p_i-\dot r)-k_pe_i.$$

The leader uses its own reference-tracking feedback. The desired motion is a translated formation $p_i=d_i+r(t)$.

## Collective behavior

Neighbor coupling suppresses differences between offset errors, while pinning removes the free translation that appears in an unpinned formation. Reference velocity and acceleration feedforward allow the nominal group to follow changing motion without relying entirely on tracking error to generate acceleration.

## Assumptions and limitations

Offsets and reference derivatives must be mutually consistent and known where used. Positive damping alone does not establish stability for an arbitrary directed graph; tracking modes must be stabilized by the selected gains. The double-integrator model permits planar acceleration directly. Inter-agent spacing is a formation objective, not a separate collision-avoidance guarantee.
