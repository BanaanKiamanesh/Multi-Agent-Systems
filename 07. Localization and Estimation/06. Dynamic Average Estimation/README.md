# Dynamic Average Estimation

Dynamic average estimation lets each agent track the average of locally observed signals. Neighbor communication combines proportional disagreement damping with an integral network state.

## Mathematical formulation

For local inputs u, estimates x and auxiliary states w,

$$\dot x=-\gamma x-k_pLx+k_iL^Tw+\gamma u,\qquad
\dot w=-k_iLx.$$

The input term pulls each estimate toward its local signal, while the coupled proportional–integral terms distribute information across the network.

## Collective behavior

On a connected undirected graph, averaging gives $\dot{\bar x}=\gamma(\bar u-\bar x)$. Therefore even the collective estimate follows a first-order filter of the true average. Constant inputs permit asymptotically exact agreement at their average; changing inputs generally produce lag unless further signal-model information is supplied.

## Assumptions and limitations

Positive gains and the appropriate connected graph are assumed. Integral action removes steady disagreement for constant inputs, but it does not make arbitrary moving-average tracking exact. Signal speed relative to estimator bandwidth controls transient error. Initial integral components in the graph nullspace do not affect the estimate through the Laplacian coupling.
