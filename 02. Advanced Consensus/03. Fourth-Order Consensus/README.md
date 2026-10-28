# Fourth-Order Consensus

Fourth-order consensus extends agreement to agents with position, velocity, acceleration and jerk states. Laplacian feedback acts on position while absolute damping acts on its higher derivatives.

## Mathematical formulation

The dynamics obey

$$q^{(4)}=-g_0Lq-g_1\dot q-g_2\ddot q-g_3q^{(3)}.$$

For each graph eigenvalue $\lambda\ne0$, the characteristic polynomial is $s^4+g_3s^3+g_2s^2+g_1s+g_0\lambda$.

## Collective behavior

For real positive $\lambda$, positive coefficients must additionally satisfy $g_3g_2>g_1$ and $g_3g_2g_1>g_1^2+g_3^2g_0\lambda$. These Routh–Hurwitz conditions explain why increasing the position gain can destabilize an otherwise damped network. Stable modes lead to position agreement and vanishing higher derivatives.

## Assumptions and limitations

The graph must support agreement, and all disagreement modes must be stable. The zero-mode polynomial after removing its free position integrator must also be stable. Complex eigenvalues require direct root analysis. The final position may shift because of nonzero initial derivatives; finite-step integration must resolve the fastest damped modes.
