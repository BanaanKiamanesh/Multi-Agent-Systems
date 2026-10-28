# Riccati Linear Synchronization

Riccati synchronization aligns identical linear dynamical agents through state differences. A stabilizing algebraic Riccati equation supplies a feedback direction adapted to the common plant.

## Mathematical formulation

For $\dot x_i=Ax_i+Bu_i$, solve

$$A^TP+PA-PBB^TP+I=0,\qquad K=B^TP.$$

The distributed feedback is $u_i=-cK\sum_j a_{ij}(x_i-x_j)$. Each disagreement mode has matrix $A-c\lambda BK$.

## Collective behavior

On a connected undirected graph, the sufficient bound $c>1/(2\lambda_2)$ makes a common P-based modal Lyapunov derivative negative. Differences between agent states decay even when the synchronized collective trajectory continues evolving under A. Synchronization therefore need not mean that every state approaches zero.

## Assumptions and limitations

The plant pair (A,B) must be stabilizable and a stabilizing positive Riccati solution must exist. Agents share the same plant, full-state feedback and a connected positive-weight undirected communication graph. This is nominal linear synchronization; it does not by itself provide robust H-infinity disturbance attenuation, output-only estimation or protection from actuator saturation.
