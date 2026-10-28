# Distributed Rigidity Eigenvector

Distributed rigidity eigenvector estimation identifies the weakest non-rigid deformation mode of a planar framework. It combines shifted power iteration with neighbor gossip to approximate global scalar reductions.

## Mathematical formulation

Let $M=R^TR$, where R is the distance rigidity matrix. After removing the two translation modes and the rotation mode, iterate

$$z^{\ell+1}\propto(I-\eta M)z^\ell.$$

Metropolis gossip approximates projection coefficients, vector norms and Rayleigh quotients. The desired eigenvalue estimate is $z^TMz$ for a normalized vector.

## Collective behavior

A suitable shift makes the smallest positive eigenvalue correspond to the largest retained iteration multiplier. Deflation prevents the rigid-motion nullspace from dominating. The eigenvector describes the deformation direction with least distance-constraint stiffness, while the eigenvalue quantifies the associated rigidity margin.

## Assumptions and limitations

The planar framework is fixed and known in a common frame. A nonzero initial component in the desired eigenspace and a suitable spectral gap are needed. Finite gossip rounds make projection and normalization approximate. A global matrix-norm bound sets the iteration step, so the overall procedure is not entirely local. Degenerate or flexible frameworks invalidate the simple three-null-mode interpretation.
