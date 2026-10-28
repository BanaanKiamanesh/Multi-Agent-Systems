# Multi-Leader Containment

Multi-leader containment brings followers into the convex hull of stationary leader positions. The objective is inclusion in a region defined collectively by the leaders, rather than agreement at a single point.

## Mathematical formulation

Partition the Laplacian into follower and leader blocks. With leaders fixed,

$$\dot p_f=-k(L_{ff}p_f+L_{fl}p_l),\qquad \dot p_l=0.$$

If the follower block is nonsingular and positive stable, its equilibrium is $p_f^*=-L_{ff}^{-1}L_{fl}p_l$.

## Collective behavior

For nonnegative graph weights and leader reachability, the equilibrium matrix has nonnegative entries and row sums equal to one. Each follower equilibrium is therefore a convex combination of leader positions. Different followers may settle at different points inside the leader hull.

## Assumptions and limitations

Every follower must be reachable from at least one leader through positive-weight directed edges. A follower-only component cannot be contained by leaders it never receives information from. Leaders remain stationary in this formulation. Finite-time positions can lie outside the hull during convergence, and containment does not impose follower separation or obstacle avoidance.
