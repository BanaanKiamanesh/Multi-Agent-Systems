# Proximal Relative Estimation

Proximal relative estimation solves the anchored displacement least-squares problem through synchronous local updates. A diagonal regularization term tempers each update without requiring a global matrix inverse.

## Mathematical formulation

Let A be the unit-weight adjacency, $d_i$ the degree and $b_i$ the signed sum of incident displacement measurements. The update is

$$p_i^{\ell+1}=\frac{2\sum_j a_{ij}p_j^\ell+\rho p_i^\ell+2b_i}{2d_i+\rho},\qquad \rho>0.$$

The anchor estimate is replaced by its known position at every round.

## Collective behavior

The neighbor sum exchanges spatial estimates, the forcing term incorporates measurements and the rho term retains part of the previous estimate. At equilibrium the regularization cancels, leaving the anchored least-squares normal equations. Thus rho affects the transient convergence rate rather than changing the intended stationary solution.

## Assumptions and limitations

The graph must be connected, undirected and unit weighted for this update. Measurements must share a common coordinate frame and orientation convention. Positive regularization and valid anchoring are required. Inconsistent relative measurements lead to a least-squares compromise, and a finite communication budget leaves nonzero estimation error.
