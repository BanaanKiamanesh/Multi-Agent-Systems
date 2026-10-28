# Centroidal Voronoi Coverage

Centroidal Voronoi coverage distributes agents across a region so that important locations are close to an agent. A spatial density assigns greater importance to areas requiring stronger sensing coverage.

## Mathematical formulation

The Voronoi cell $V_i$ contains points closer to $p_i$ than to any other agent. Its density-weighted centroid is

$$c_i=\frac{\int_{V_i}q\rho(q)\,dq}{\int_{V_i}\rho(q)\,dq},\qquad \dot p_i=k(c_i-p_i).$$

A finite sensing radius restricts the mass contributing to the centroid; an empty cell leaves its agent stationary.

## Collective behavior

For fixed density and unlimited sensing, moving toward centroids decreases the locational objective $H=\int\min_i\|q-p_i\|^2\rho(q)\,dq$. With finite radius R, the capped objective uses $\min(\min_i\|q-p_i\|,R)^2$. Uncovered points contribute $R^2$ instead of disappearing from the objective.

## Assumptions and limitations

The domain is represented by weighted spatial samples, so cell membership changes discretely and the cost is piecewise smooth. Positive gain and nonnegative density are required. Coverage equilibria are generally local: different initial placements can yield different partitions and costs. The coverage objective does not itself enforce collision avoidance or motion constraints.
