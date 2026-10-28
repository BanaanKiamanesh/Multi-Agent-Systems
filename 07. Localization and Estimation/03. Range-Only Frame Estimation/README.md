# Range-Only Frame Estimation

Range-only frame estimation reconstructs a network of agent positions from inter-agent distances. Known anchors determine a reference frame, while distance residuals constrain the unknown geometry.

## Mathematical formulation

For measured distance $d_e$ on edge e=(i,j), define

$$r_e(p)=\frac12(\|p_i-p_j\|^2-d_e^2),\qquad
F(p)=\frac12\|r(p)\|^2.$$

The residual Jacobian R is a rigidity matrix. Gradient motion is $\dot p=-kR^Tr$, with anchor positions fixed.

## Collective behavior

Each edge pushes its endpoints to reduce a squared-distance mismatch. The objective is nonconvex because distances lose directional information. Nearby initial estimates can converge to the intended anchored realization when the geometry is locally identifiable, while distant initializations can encounter reflected or otherwise incorrect configurations.

## Assumptions and limitations

Three noncollinear planar anchors fix a coordinate frame but do not alone make every measurement graph uniquely localizable. The range graph must provide sufficient rigidity and anchor connections. Noisy ranges need not be mutually realizable. The method is local, with sensitivity to initialization and degeneracy; zero residual is stronger evidence than merely small position updates.
