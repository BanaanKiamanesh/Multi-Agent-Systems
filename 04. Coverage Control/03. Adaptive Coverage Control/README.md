# Adaptive Coverage Control

Adaptive coverage reallocates a group of agents as the importance of different locations changes. A moving density hotspot shifts the regions where sensing effort is most valuable.

## Mathematical formulation

At time t, the centroid of agent i’s Voronoi cell is

$$c_i(t)=\frac{\int_{V_i(t)}q\rho(q,t)\,dq}{\int_{V_i(t)}\rho(q,t)\,dq},\qquad
\dot p_i=k(c_i(t)-p_i).$$

Agents therefore respond both to the geometry of their neighbors and to the changing spatial density.

## Collective behavior

For frozen density, centroid attraction has the usual coverage-descent interpretation. With moving density, the objective itself changes: its time derivative includes an explicit density-rate term in addition to motion-induced descent. The group can lag behind a rapidly moving hotspot even when the positional feedback remains stable.

## Assumptions and limitations

The density is prescribed as a function of time; “adaptive” here means adaptation of positions, not identification of unknown density parameters. Density values must remain nonnegative and weighted cell mass must be handled when zero. Initial and final costs correspond to different fields and need not decrease monotonically. Finite sensing and spatial sampling add further limitations.
