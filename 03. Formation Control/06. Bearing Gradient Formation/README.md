# Bearing Gradient Formation

Bearing-gradient formation regulates the unit directions of edges. Unlike distance control, it specifies relative angles while allowing uniform scaling of the formation.

## Mathematical formulation

For $g_{ij}=(p_j-p_i)/\|p_j-p_i\|$ and target $g_{ij}^*$, define $P_g=I-gg^T$. The edge correction is proportional to

$$\frac{1}{\|p_j-p_i\|}P_{g_{ij}}g_{ij}^*.$$

Endpoints receive opposite signs. This is the negative gradient direction of a sum of squared unit-bearing errors, with the range factor arising from differentiation of a normalized vector.

## Collective behavior

The projection removes the component along the current edge, rotating its direction rather than directly prescribing its length. Pairwise antisymmetry preserves the centroid, and orthogonality preserves centered scale in ideal continuous time. Bearing rigidity determines whether the target bearings identify shape up to translation and scale.

## Assumptions and limitations

Bearings must be realizable in a common orientation frame, and edge endpoints must remain distinct. The target framework must satisfy the appropriate infinitesimal bearing-rigidity condition. Arbitrary initial configurations are not covered by a blanket global guarantee. The inverse-distance factor becomes large near coincident endpoints and makes numerical resolution especially important.
