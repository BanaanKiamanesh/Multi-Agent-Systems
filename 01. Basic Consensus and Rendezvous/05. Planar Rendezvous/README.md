# Planar Rendezvous

Planar rendezvous applies consensus simultaneously to the two position coordinates of a group of single-integrator agents. The agents approach a shared point through local relative-position feedback.

## Mathematical formulation

For positions $p_i\in\mathbb R^2$,

$$\dot p_i=k\sum_j a_{ij}(p_j-p_i).$$

Equivalently, both coordinate columns evolve under $-kL$. A weighted directed ring makes the rendezvous location depend on asymmetric network influence.

## Collective behavior

If w is the normalized left nullvector of L, the limiting position is $p_\infty=\sum_iw_ip_i(0)$. Each coordinate obeys the same consensus dynamics. Positive mixing keeps the continuous-time positions within the convex hull of the initial positions, although agents can pass arbitrarily close to one another.

## Assumptions and limitations

The model assumes direct planar velocity actuation, a fixed strongly connected positive-weight graph and a positive gain. There are no nonholonomic steering constraints or collision-avoidance terms. Geometric rendezvous alone does not guarantee safe travel to the common point.
