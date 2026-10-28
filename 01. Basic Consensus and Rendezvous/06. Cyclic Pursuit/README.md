# Cyclic Pursuit

Cyclic pursuit organizes agents in a directed cycle. Each agent follows one designated successor, creating coordinated inward motion even though no common destination is prescribed.

## Mathematical formulation

For a cycle of n agents,

$$\dot p_i=k(p_{i+1}-p_i),\qquad p_{n+1}=p_1.$$

The associated directed-cycle Laplacian has one zero eigenvalue. Every other eigenvalue has positive real part, so the relative positions decay for $k>0$.

## Collective behavior

The balanced cycle preserves the centroid. All agents therefore rendezvous at their initial centroid. Complex eigenvalues produce rotational components in the transient motion, explaining the curved or spiraling paths of initially polygonal configurations. The final point depends on initial geometry rather than a leader command.

## Assumptions and limitations

Every successor link must remain active and the cycle must include all agents. These are unconstrained single-integrator dynamics with no obstacle or collision protection. Equal cycle gains give the stated arithmetic-centroid invariant; unequal influence generally changes the conserved weighted centroid.
