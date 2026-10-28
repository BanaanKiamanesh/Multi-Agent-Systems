# Power-Mean Consensus

Power-mean consensus reaches an agreement defined by a nonlinear average. Instead of averaging the original positive states, agents average a transformed coordinate.

## Mathematical formulation

Choose $f(x)=x^q$ for $q\ne0$, or $f(x)=\log x$ for $q=0$. The dynamics are

$$\dot x=-k\operatorname{diag}(1/f'(x))L f(x).$$

With $y_i=f(x_i)$, the chain rule gives $\dot y=-kLy$. The nonlinear network is therefore linear consensus in the transformed coordinates.

## Collective behavior

On a connected undirected graph, $\frac1n\sum_if(x_i)$ is conserved. The common limit is $(\frac1n\sum_ix_i(0)^q)^{1/q}$ for nonzero q, and $\exp(\frac1n\sum_i\log x_i(0))$ at q=0. The ordinary arithmetic mean corresponds to q=1.

## Assumptions and limitations

States must remain strictly positive so the transformation and its inverse are valid. Positive gain and connected nonnegative coupling are required. A numerical step can violate positivity even when the transformed continuous dynamics remain well behaved; the exponent and step size should be chosen accordingly. Directed coupling generally yields a weighted transformed mean.
