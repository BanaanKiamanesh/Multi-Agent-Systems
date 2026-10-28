# Distance-Based Formation

Distance-based formation regulates lengths of selected inter-agent edges. Desired shape is specified by distances, leaving the overall position and orientation free.

## Mathematical formulation

For edge e joining i and j, let $r_e=\|p_i-p_j\|$ and let $d_e>0$ be its target length. The potential is

$$V(p)=\frac{k}{2}\sum_e(r_e-d_e)^2.$$

Each endpoint receives the negative gradient of its incident edge costs. An edge contributes a radial force proportional to $(r_e-d_e)(p_i-p_j)/r_e$.

## Collective behavior

Edges longer than desired attract; shorter edges repel. Equal and opposite endpoint forces preserve the centroid. Any rigid translation or rotation of a correct formation has the same zero cost. Locally rigid target geometry removes nontrivial small shape deformations, but reflection ambiguities and undesired equilibria can remain.

## Assumptions and limitations

Distance constraints must be geometrically realizable and the target framework must have the required rigidity. Coincident endpoints make radial directions undefined; regularization near zero distance does not create a global convergence proof. The nonconvex potential supports local rather than universal shape recovery, and nonadjacent collisions are not separately prevented.
