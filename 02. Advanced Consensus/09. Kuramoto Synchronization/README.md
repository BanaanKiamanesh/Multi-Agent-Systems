# Kuramoto Synchronization

Kuramoto synchronization couples oscillator phases rather than positions. Neighboring oscillators influence one another through the sine of their phase difference.

## Mathematical formulation

For natural frequencies $\omega_i$,

$$\dot\theta_i=\omega_i+k\sum_j a_{ij}\sin(\theta_j-\theta_i).$$

The coupling is periodic, so phases differing by an integer multiple of $2\pi$ represent the same oscillator orientation. A common natural frequency can be removed by a rotating reference frame.

## Collective behavior

When phases are cohesive, the sine interaction is locally attractive and resembles linear consensus for small differences. Identical frequencies allow phase agreement under suitable connectivity and initial conditions. Heterogeneous frequencies can instead produce frequency locking with persistent phase offsets; insufficient coupling may fail to lock the network.

## Assumptions and limitations

The identical-frequency, initially cohesive case is more restrictive than arbitrary oscillator synchronization. Graph connectivity and positive coupling are necessary ingredients but do not guarantee a unique globally attracting phase pattern. Phase wrapping must be distinguished from actual frequency disagreement when interpreting motion.
