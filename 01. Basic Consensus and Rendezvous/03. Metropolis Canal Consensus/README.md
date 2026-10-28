# Metropolis Canal Consensus

Metropolis averaging reaches agreement without requiring agents to know the whole graph spectrum. A canal network supplies a useful graph interpretation: canal segments become agents, and segments meeting at a junction become neighbors in the corresponding line graph.

## Mathematical formulation

For neighboring agents,

$$w_{ij}=\frac{1}{1+\max(d_i,d_j)},\qquad w_{ii}=1-\sum_{j\ne i}w_{ij}.$$

Nonneighbors have zero weight, and the update is $x^{\ell+1}=Wx^\ell$. Only neighbor degrees are needed to define each off-diagonal weight.

## Collective behavior

The weights are symmetric and their rows sum to one. Their positive diagonal prevents periodic alternation, while connectedness lets information reach the entire network. The limiting value is the initial arithmetic mean, and disagreement decreases through repeated local mixing.

## Assumptions and limitations

The communication graph must be connected and undirected. The line-graph construction concerns coordination between canal segments; these scalar updates do not describe water flow, storage balances, gate dynamics or hydraulic constraints. Changing the topology requires recalculating degrees and mixing weights.
