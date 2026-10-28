# Rigidity Maintenance

Rigidity maintenance protects a formation from losing its local shape constraints. It uses the smallest nontrivial eigenvalue of the planar rigidity Gram matrix as a geometric margin.

## Mathematical formulation

Let $M(p)=R(p)^TR(p)$ and let $\lambda_4$ be its first eigenvalue after the three rigid-motion modes. A barrier is

$$B(p)=\frac{\beta}{\lambda_4(p)-\lambda_{\min}}.$$

Combined with a target-position objective, the feedback adds $\beta\nabla\lambda_4/(\lambda_4-\lambda_{\min})^2$ to the target attraction.

## Collective behavior

A larger positive rigidity margin resists nearly flexible deformations. The barrier pushes away from shapes whose weakest constrained mode approaches the chosen threshold. If the target geometry is too flat or otherwise incompatible with the required margin, the equilibrium must trade tracking accuracy for rigidity.

## Assumptions and limitations

The initial framework must lie strictly above the threshold. The eigenvalue-gradient formula assumes a simple eigenvalue; crossings can make the selected eigenvalue nonsmooth. The eigenpair calculation is centralized. Continuous barrier behavior does not certify finite-step constraint preservation, and rigidity maintenance alone does not ensure collision avoidance.
