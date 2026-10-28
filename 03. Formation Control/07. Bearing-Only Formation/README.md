# Bearing-Only Formation

Bearing-only formation coordinates agents using edge directions without requiring range measurements in the feedback magnitude. Desired bearings specify shape up to translation and scale.

## Mathematical formulation

Let $g_{ij}$ be the current unit direction from i to j and $g_{ij}^*$ the target. The edge feedback uses

$$P_{g_{ij}}g_{ij}^*,\qquad P_g=I-gg^T,$$

with equal and opposite endpoint contributions. Unlike the range-aware bearing gradient, this law omits the inverse edge-length factor.

## Collective behavior

The correction lies perpendicular to the current edge. The resulting antisymmetric interactions preserve the centroid and centered scale in continuous time. Correct bearings make every projected error vanish. Bearing rigidity excludes infinitesimal shape deformations other than translation and scaling, but other equilibria still require attention.

## Assumptions and limitations

Agents need a compatible orientation frame for interpreting bearings, and coincident edge endpoints are inadmissible. The target bearings must be consistent and bearing rigid. Omitting range does not remove the geometric singularity at zero separation. Continuous invariants may drift under finite integration steps, and the law alone does not protect against every inter-agent collision.
