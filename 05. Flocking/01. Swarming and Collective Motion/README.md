# Swarming and Collective Motion

Swarming combines short-range repulsion and longer-range attraction to form a coherent group. Velocity damping dissipates motion so the swarm can settle around a balance of pair forces.

## Mathematical formulation

For pair distance $r_{ij}$, a Morse-type force magnitude is

$$f(r)=c_re^{-r/\ell_r}-c_ae^{-r/\ell_a}.$$

An agent’s acceleration sums weighted radial forces $a_{ij}f(r_{ij})(p_i-p_j)/r_{ij}$ and adds $-cv_i$. Short-range repulsion and longer-range attraction compete to select characteristic separations.

## Collective behavior

Pairwise interactions create collective geometry without assigning every edge a desired length. On a symmetric graph, internal forces cancel in the total momentum balance; damping removes collective momentum. Local force equilibria depend on network connectivity, the attraction–repulsion balance and initial placement.

## Assumptions and limitations

The repulsion range should be shorter than the attraction range for the intended cohesive behavior. Positive damping reduces persistent oscillation but does not guarantee one globally optimal swarm configuration. Finite force magnitudes and regularized zero-distance directions do not establish collision avoidance. Disconnected interaction groups can settle independently.
