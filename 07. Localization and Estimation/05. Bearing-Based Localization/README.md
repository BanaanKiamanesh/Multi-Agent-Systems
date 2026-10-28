# Bearing-Based Localization

Bearing-based localization estimates agent coordinates from known edge directions and fixed anchor positions. It exploits linear projection constraints instead of nonlinear distance residuals.

## Mathematical formulation

For a measured unit bearing $g_{ij}$, the relative position must satisfy

$$(I-g_{ij}g_{ij}^T)(p_j-p_i)=0.$$

Summing these projector constraints gives a block bearing Laplacian B. Free estimates evolve as $\dot p_f=-k(B_{ff}p_f+B_{fa}p_a)$ while anchors remain fixed.

## Collective behavior

If the anchored follower block is positive definite, the equilibrium is unique and exponentially attractive. Information propagates through bearing constraints to determine positions jointly. Anchors remove translation and scale ambiguities that a bearing-only network would otherwise retain.

## Assumptions and limitations

Bearings and anchor coordinates must be compatible in a common orientation frame. Having two anchors is not sufficient by itself; the complete anchored graph must be localizable. Projectors constrain lines and cannot distinguish a bearing from its negative independently. Inconsistent measurements may yield a compromise rather than an exact geometric realization.
