# Heterogeneous Fractional Network

A heterogeneous fractional network combines agents with different memory orders, intrinsic scalar dynamics and signed directed interactions. It studies collective behavior when agents do not share one common temporal model.

## Mathematical formulation

Agent i obeys a relation of the form

$$D^{\alpha_i}x_i=b(t)x_i+k\sum_j a_{ij}(x_j-x_i),$$

where $b(t)=0.18\sin(0.6t)-0.33$ and the edge weights may be signed. Each agent has its own initialized memory kernel.

## Collective behavior

The intrinsic term acts on individual states while the coupling transfers influence between them. Different fractional orders create different response histories even under identical instantaneous inputs. Signed directed coupling may amplify some disagreements instead of damping them, and the ordinary average is not generally an invariant.

## Assumptions and limitations

This model lies outside standard identical-agent positive-weight consensus assumptions. A common equilibrium or visually shrinking disagreement is not a universal synchronization certificate. Orders must be admissible and assigned consistently to the agents. Stability depends on the combined intrinsic dynamics, signed graph and fractional memory; finite-history numerical accuracy also depends on the time step.
