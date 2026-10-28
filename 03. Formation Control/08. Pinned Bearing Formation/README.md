# Pinned Bearing Formation

Pinned bearing formation uses stationary anchors to remove the free translation and scale of a bearing-defined shape. Followers respond to linear bearing-line constraints relative to their neighbors.

## Mathematical formulation

For each desired unit bearing $g_e^*$, form $P_e=I-g_e^*g_e^{*T}$. These projectors assemble a block bearing Laplacian B. With anchor positions $p_a$ fixed,

$$\dot p_f=-k(B_{ff}p_f+B_{fa}p_a).$$

The anchor positions act as boundary values rather than free states.

## Collective behavior

If the follower block is positive definite, the follower equilibrium is unique and exponentially stable. The anchor geometry fixes the otherwise undetermined translation and size. Each projector imposes alignment with a line: its kernel includes both directions along that line, so oriented-bearing agreement requires compatible global geometry.

## Assumptions and limitations

Two anchors are not automatically sufficient; the actual anchored framework must make the follower block positive definite. Anchor coordinates and desired bearings must be consistent. The linear law assumes unconstrained planar velocity actuation. It regulates shape geometry but does not enforce speed limits, obstacle clearance or collision-free transients.
