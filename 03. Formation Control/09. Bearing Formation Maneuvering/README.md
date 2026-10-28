# Bearing Formation Maneuvering

Bearing formation maneuvering changes a formation’s translation and scale while preserving its bearing pattern. Leaders prescribe compatible velocities, and followers use proportional–integral correction.

## Mathematical formulation

With bearing Laplacian B and follower integral state z,

$$\dot p_f=-k_p(Bp)_f-k_iz_f,\qquad \dot z_f=(Bp)_f.$$

A compatible target has $p_i^*(t)=s(t)p_i^0+c(t)$, so $\dot p_i^*=\dot s\,p_i^0+\dot c$. Uniform translation and positive scaling leave bearings unchanged.

## Collective behavior

The proportional term removes geometric error. Integral action supplies the nonzero follower velocities needed for sustained motion even when bearing error tends to zero. Multiple leaders can jointly determine translation and scale; their commands must describe one coherent maneuver rather than competing shape changes.

## Assumptions and limitations

The pinned follower block must be positive definite and gains positive. The nominal maneuver uses compatible constant leader velocities, corresponding to affine translation and scale over time. The scale must stay positive. Arbitrary leader acceleration or inconsistent leader commands are outside this model’s tracking claim. No separate collision or actuator constraints are enforced.
