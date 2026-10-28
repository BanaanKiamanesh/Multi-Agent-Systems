# First-Order Undirected Consensus

First-order consensus brings scalar agent states to a common value using only differences with neighboring states. It models agreement of sensor readings, distributed estimates, or coordination variables on an undirected communication network.

## Mathematical formulation

For positive edge weights $a_{ij}=a_{ji}$, define the Laplacian $L=D-A$, where $D_{ii}=\sum_j a_{ij}$. The dynamics are

$$\dot x_i=k\sum_j a_{ij}(x_j-x_i),\qquad \dot x=-kLx.$$

The gain $k>0$ sets the response speed. Each agent moves toward its neighbors' weighted state values.

## Collective behavior

Symmetry gives $\mathbf1^TL=0$, so the arithmetic mean is conserved. On a connected graph, every nonconstant Laplacian mode decays and $x_i(t)\to \frac1n\sum_jx_j(0)$. The slowest disagreement mode decays at rate $k\lambda_2(L)$; stronger algebraic connectivity generally accelerates agreement.

## Assumptions and limitations

Communication must be connected through positive-weight edges. Disconnected components reach separate agreements. The mean-preservation statement requires symmetric or suitably balanced coupling; it does not automatically apply to a row-normalized matrix. A finite integration step must resolve the fastest mode as well as the slowest one.
