# Olfati-Saber Algorithm 1

Olfati-Saber Algorithm 1 forms a free flock using local spacing and velocity alignment, without a navigation leader. Agents organize around a preferred spacing while their collective motion remains unpinned.

## Mathematical formulation

The local acceleration is

$$\dot v_i=\sum_j\phi_\alpha(\|p_j-p_i\|_\sigma)n_{ij}
+\sum_j a_{ij}(p)(v_j-v_i).$$

Sigma-distance smooths the Euclidean norm; a smooth finite-range weight defines neighbors. The shaping function changes sign at the desired spacing.

## Collective behavior

Spacing forces descend a collective potential, while velocity differences dissipate relative kinetic energy. Symmetric internal interactions preserve total momentum, so the centroid can continue moving at its initial mean velocity. Without leader navigation, spatially separated subgroups may fragment rather than unite into a single flock.

## Assumptions and limitations

The desired distance lies strictly inside the sensing radius, with valid smoothing parameters and nonnegative gains. The force shape uses parameters satisfying $0<A\le B$ to place its zero at the requested distance. Time-varying connectivity and finite forces require separate consideration for collision avoidance and global convergence. No absolute position target selects the final flock location.
