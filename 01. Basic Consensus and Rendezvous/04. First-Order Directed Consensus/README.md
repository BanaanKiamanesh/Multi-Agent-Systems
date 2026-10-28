# First-Order Directed Consensus

Directed consensus allows asymmetric information exchange: one agent may listen to another without receiving any reciprocal influence. Agreement still emerges, but the final value reflects the network influence pattern.

## Mathematical formulation

An edge $j\to i$ means that i receives j. With receiver-row adjacency,

$$L_{ii}=\sum_j a_{ij},\quad L_{ij}=-a_{ij},\quad \dot x=-kLx.$$

The directed ring provides a strongly connected communication structure with potentially unequal edge weights.

## Collective behavior

For a strongly connected positive-weight graph, a normalized positive left nullvector satisfies $w^TL=0$ and $w^T\mathbf1=1$. Thus $w^Tx$ is invariant and $x(t)\to\mathbf1w^Tx(0)$. Equal influence is recovered only under additional balance conditions.

## Assumptions and limitations

Positive gain and positive weights are assumed. Strong connectivity is sufficient for the stated strictly positive influence vector; more general rooted graphs need a different influence interpretation. Ordinary consensus preserves a weighted average, not necessarily the arithmetic mean. Reversing the edge convention changes which agents influence the result.
