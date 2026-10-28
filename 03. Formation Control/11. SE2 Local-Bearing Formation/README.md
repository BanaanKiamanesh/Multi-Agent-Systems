# SE2 Local-Bearing Formation

Local-bearing formation on SE(2) coordinates planar positions and headings. Each agent expresses observed bearings in its own orientation frame rather than relying only on world-frame directions.

## Mathematical formulation

For heading $\theta_i$, the local bearing to j is

$$b_{ij}=R(\theta_i)^T\frac{p_j-p_i}{\|p_j-p_i\|}.$$

A potential sums squared differences between these local bearings and desired local bearings in both edge directions. Position and heading rates follow its negative gradients with separate positive gains.

## Collective behavior

Position corrections change relative edge geometry; heading corrections rotate the local observation frames. A translated or positively scaled formation has the same local bearings. Rotating every position and every heading by the same angle also preserves them, so these collective freedoms cannot be eliminated by local-bearing error alone.

## Assumptions and limitations

The model assumes fully actuated planar translation and rotation, plus the relative-heading information needed to relate frames. It is not a nonholonomic vehicle model. Desired local bearings must be realizable and edge endpoints noncoincident. The joint position–orientation objective is nonconvex, so general initial conditions do not guarantee global convergence.
