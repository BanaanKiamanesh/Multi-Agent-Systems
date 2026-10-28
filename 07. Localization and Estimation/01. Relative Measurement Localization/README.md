# Relative Measurement Localization

Relative-measurement localization estimates agent positions from noisy or inconsistent pairwise displacement measurements. A known anchor removes the otherwise arbitrary global translation.

## Mathematical formulation

For oriented edge i→j, let $z_{ij}$ measure $p_j-p_i$. The least-squares objective is

$$F(p)=\frac12\sum_{(i,j)}\|p_j-p_i-z_{ij}\|^2.$$

Each residual contributes equal and opposite gradient corrections to its endpoints. The anchor position is held fixed while followers descend F.

## Collective behavior

With consistent measurements, the anchored equilibrium recovers the network geometry. With inconsistent cycle measurements, the residual cannot vanish on every edge; the solution balances those errors in a least-squares sense. Relative measurements determine the shape jointly, so every estimate depends on information propagated through the graph.

## Assumptions and limitations

The measurement graph must connect each unknown agent to the anchor. Displacements must be expressed in a compatible coordinate frame and use a consistent edge orientation. A fixed anchor determines translation but does not correct systematic frame errors in the measurements. The quadratic problem is well posed under anchored connectivity; changing or moving measurements require a dynamic extension.
