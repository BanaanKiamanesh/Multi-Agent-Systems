# Henneberg Rigid Formation

Henneberg formation combines a minimally rigid planar communication graph with distance-gradient motion. The graph is built incrementally so a small number of inter-agent distance constraints can determine the shape locally.

## Mathematical formulation

A Henneberg I move adds one vertex and two incident edges. A Henneberg II move removes an existing edge, adds a new vertex, and connects it to that edge's endpoints and a third vertex. Starting from one edge, these moves preserve generic minimal rigidity and the edge count $2n-3$.

Positions then descend $V=\frac{k}{2}\sum_e(\|p_i-p_j\|-d_e)^2$.

## Collective behavior

The construction addresses the combinatorial sufficiency of the distance graph. The gradient law addresses the geometric realization of those distances. Together they yield sparse shape regulation while retaining the unavoidable translation and rotation freedoms. Sparse constraints do not imply a unique global realization for every initial condition.

## Assumptions and limitations

Rigidity is generic: a particular collinear or degenerate placement can lose rank despite a valid Henneberg graph. Desired lengths must arise from a realizable nondegenerate framework. Distance-gradient dynamics remain nonconvex and can reach undesired equilibria. Minimal rigidity is a local property, not a global collision or convergence certificate.
