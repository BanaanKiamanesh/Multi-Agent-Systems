# Shrinking-Horizon Preference Agreement

Shrinking-horizon preference agreement repeatedly replans as agents move toward a negotiated terminal state. It limits communication at each physical step instead of fully converging the terminal agreement problem beforehand.

## Mathematical formulation

At each time step, agents solve local quadratic integrator plans over the remaining horizon, perform a fixed number of neighbor-based multiplier updates,

$$\lambda\leftarrow\lambda+\eta Lx_N,$$

and execute only the first planned input. The horizon then shrinks, and multipliers are reused as the initial dual state for the next negotiation.

## Collective behavior

Reusing multipliers carries information from previous communication rounds. Replanning incorporates the newly reached physical states. A small communication budget reduces negotiation effort but can leave sizeable differences in predicted terminal states, particularly as the remaining horizon becomes short and corrections become expensive.

## Assumptions and limitations

Positive state/input weights and a connected communication graph are required. Finite communication sweeps do not enforce exact terminal equality, and warm starts do not remove that limitation. The local dynamics use a unit discrete step. The method is not a guarantee of recursively feasible constrained model-predictive control; state and actuator constraints are absent.
