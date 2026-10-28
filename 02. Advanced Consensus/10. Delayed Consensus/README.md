# Delayed Consensus

Delayed consensus uses an earlier network state to determine the current rate of agreement. Communication delay introduces phase lag and can destabilize a network that is stable without delay.

## Mathematical formulation

For a common delay $\tau\ge0$,

$$\dot x(t)=-kLx(t-\tau).$$

A constant history specifies all states before time zero. For real positive disagreement eigenvalues, each mode obeys $\dot z(t)=-k\lambda z(t-\tau)$.

## Collective behavior

Continuous disagreement stability requires $k\lambda_{\max}\tau<\pi/2$ for a connected undirected positive-weight graph, using the Laplacian actually governing the dynamics. The same real-mode condition applies to its row-normalized Laplacian. Larger gains accelerate delay-free agreement but reduce the admissible delay.

## Assumptions and limitations

The delay is common to the complete Laplacian feedback, including its self term. Other delay placements have different characteristic equations. A time grid rounds the delay to a whole number of steps, and forward-Euler integration imposes a further stability restriction. With zero delay the dynamics reduce to ordinary consensus; a continuous-time bound alone is insufficient for a coarse discretization.
