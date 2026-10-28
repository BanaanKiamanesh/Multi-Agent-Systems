# Effective Resistance Robustness

Effective-resistance robustness quantifies how much an attractive consensus network can tolerate an additional repulsive interaction. The positive network’s alternate paths determine the strength of that tolerance.

## Mathematical formulation

For incidence vector $b=e_i-e_j$ and a connected positive base Laplacian L,

$$R_{\rm eff}=b^TL^+b,\qquad L_{\rm pert}=L-db b^T.$$

Here d>0 is the magnitude of a negative edge and $L^+$ is the Laplacian pseudoinverse. Strict stability of disagreement requires $dR_{\rm eff}<1$.

## Collective behavior

Low effective resistance means the positive network offers strong alternative routes between the affected endpoints. Such pairs tolerate a larger negative edge. At the threshold an additional undamped disagreement direction appears; beyond it a negative Laplacian eigenvalue produces growing disagreement under the nominal consensus law.

## Assumptions and limitations

The base graph must be connected, undirected and positively weighted. The scalar threshold concerns one negative-edge perturbation. Multiple simultaneous perturbations interact and need a coupled matrix condition. Agreement retains a free mean mode, so strict stability refers only to disagreement. The continuous criterion must still be paired with an adequate integration step.
