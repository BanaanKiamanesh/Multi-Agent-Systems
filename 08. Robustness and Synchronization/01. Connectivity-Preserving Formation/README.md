# Connectivity-Preserving Formation

Connectivity-preserving formation balances desired agent positions against the need to retain communication links. A barrier grows as any protected edge approaches its communication range.

## Mathematical formulation

For edge length $r_e<R$, use the barrier

$$B_e=\beta\frac{r_e^2}{R^2-r_e^2}.$$

The total potential adds a quadratic target-position cost. Its negative gradient combines target attraction with equal and opposite edge forces whose magnitude grows sharply near the range boundary.

## Collective behavior

The barrier discourages stretching protected links beyond R. Since target attraction and link preservation may conflict, the final formation generally compromises rather than attaining all desired positions exactly. Maintaining a connected protected subgraph maintains graph connectivity, even if other optional communication links change.

## Assumptions and limitations

The initial protected graph must be connected and every protected edge strictly shorter than the communication radius. The barrier reasoning is continuous-time and assumes unbounded admissible gradient action near the boundary. Finite temporal steps or actuator limits can violate that ideal argument. The barrier prevents excessive separation, not collisions between agents.
