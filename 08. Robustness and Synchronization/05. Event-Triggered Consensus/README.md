# Event-Triggered Consensus

Event-triggered consensus reduces communication by broadcasting only when an agent’s current state differs sufficiently from its last transmitted value. Neighbors use held values between broadcasts.

## Mathematical formulation

Agent i transmits when

$$|x_i-\hat x_i|\ge ce^{-\delta t}+\varepsilon,$$

and then resets $\hat x_i=x_i$. Its control is $\dot x_i=-k(d_ix_i-\sum_j a_{ij}\hat x_j)$. Trigger conditions are evaluated at periodic checking times.

## Collective behavior

Initially frequent transmissions can become less frequent as disagreement shrinks. The exponential threshold tightens over time, while the positive floor prevents arbitrarily small deviations from continually causing events. Because current self states are mixed with held neighbor states, the ordinary arithmetic-mean invariant is generally lost.

## Assumptions and limitations

This is periodic event checking rather than continuous event detection. A positive threshold floor supports practical agreement with residual error rather than a general exact-consensus claim. Communication graph, gain, checking period and threshold parameters jointly affect stability and communication frequency. Packet loss, variable transmission delay and quantization require additional modeling.
