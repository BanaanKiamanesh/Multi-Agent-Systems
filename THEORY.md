# Theory and implementation guide

This guide describes the equations implemented in the standalone algorithm folders.
Lesson numbers match the [learning path](LEARNING_PATH.md). Results depend on the
stated assumptions; numerical trajectories alone establish neither general
stability nor equivalence to the original course software.

## Graph convention and numerical integration

An edge `j -> i` means that agent i receives agent j. Adjacency has receiver
rows, so `L = diag(A 1) - A` and `(L x)_i = sum_j a_ij (x_i-x_j)`.
`WeightedLaplacian` accepts finite nonnegative weights. Signed examples construct
signed interactions separately. Zero-weight edges do not provide communication.

`NormalizedLaplacian` returns `D^-1 L`, leaving isolated rows zero. On an
undirected connected graph this preserves a degree-weighted average, not usually
the arithmetic mean. Graph connectivity and weighted connectivity coincide only
when all structural edges have positive weight.

Smooth ordinary differential equations use fixed-step RK4. Delay equations use
forward Euler and constant prehistory. Fractional equations use initialized
Grunwald–Letnikov memory on `x-x(0)` and explicit forcing from the previous step.
All three integrators end exactly at the requested horizon. A vector of fractional
orders follows NumPy broadcasting to the state shape. The fractional method has
quadratic cost in the number of time samples and does not preserve RK4 accuracy.

Neither positive gains nor a continuous-time proof guarantees numerical stability.
Switches, non-Lipschitz feedback, grid-cell changes and clipping reduce the usual
smooth-RK4 accuracy. Barrier controllers reject invalid sampled states; a finite
step is not a continuous collision or constraint certificate.

## 1–7. Consensus, rendezvous and message propagation

First-order consensus is `xdot = -k L x`. A connected positive-weight undirected
network converges to its initial arithmetic mean. For a strongly connected
directed network, the conserved average is `w^T x`, with `w^T L=0`, `w^T 1=1`.
The weighted directed-ring rendezvous applies this law to both planar coordinates.
Cyclic pursuit uses successor edges and preserves centroid on its balanced ring.
These distinctions follow the [consensus analysis of Olfati-Saber and Murray](https://www.ee.iitb.ac.in/~dc/EE749/Olfati.pdf).

Discrete consensus uses `x[k+1]=(I-hL)x[k]`. The enforced sufficient condition
`0<h<1/max_i L_ii` makes mixing nonnegative with positive diagonal. Convergence
also requires a rooted positive-weight communication graph. Metropolis averaging
uses symmetric off-diagonal weights `1/(1+max(deg_i,deg_j))` and a diagonal chosen
for row sums of one. The canal example applies this to a line graph, not a hydraulic
plant. Max/min consensus propagates neighbor extrema, including self; on a
connected undirected graph it finishes within the graph diameter.

## 8–19. Higher-order and nonlinear agreement

For an mth-order integrator, the position uses Laplacian feedback while higher
derivatives use absolute damping. Each nonzero Laplacian eigenvalue lambda has
characteristic polynomial

```text
s^m + g[m-1] s^(m-1) + ... + g[1] s + g[0] lambda.
```

The implementation checks modal roots, including damping of the zero mode after
removing its free position integrator. For real positive lambda, third-order
stability requires `g[2] g[1] > g[0] lambda`. Fourth-order stability additionally
requires `g[3] g[2] > g[1]` and
`g[3] g[2] g[1] > g[1]^2 + g[3]^2 g[0] lambda`, with positive coefficients.
Complex directed modes need their actual roots checked. Collective position may
shift when initial derivative states are nonzero. Linear state-space consensus
adds relative-position/velocity feedback and absolute velocity damping.

Nonlinear leader tracking uses tanh differences, pinning and reference-rate
feedforward. The designated leader must start at its prescribed reference.
Switching consensus cycles through connected undirected topologies, sharing an
arithmetic-mean invariant and a disagreement Lyapunov function.

Finite-time consensus uses antisymmetric `sign(delta)|delta|^alpha`, `0<alpha<1`.
The continuous-time undirected positive-weight system reaches agreement in finite
time; fixed-step integration generally leaves a residual. Power-mean consensus
uses `xdot=-k diag(1/f'(x)) L f(x)` on positive states, with `f(x)=x^q` or `log(x)`.
The invariant is the mean of f(x); the nonlinear coordinate transformation is
valid only while states remain positive.

Signed nonlinear consensus is the gradient flow of
`k sum_e w_e log(cosh(x_i-x_j))`. Negative weights can destroy convexity and
stability; this is not the structurally balanced bipartite-consensus protocol.
Kuramoto coupling uses `omega_i+k sum_j a_ij sin(theta_j-theta_i)`. The preset has
identical frequencies and initially cohesive phases. General frequencies can
produce phase offsets, and arbitrary phase configurations need not synchronize.

Delayed consensus uses `xdot=-k L x(t-tau)` with delay rounded to the actual grid.
For an undirected connected positive-weight graph, both its ordinary Laplacian
and its row-normalized Laplacian have real nonnegative eigenvalues. Continuous
disagreement stability requires `k lambda_max tau < pi/2`, using the matrix actually
simulated; the Euler discretization imposes further step limits. The delayed preset
saves its effective grid-rounded delay and gain alongside that matrix.
The fractional tanh-of-Laplacian example does not generally preserve the arithmetic
mean. The heterogeneous fractional example adds distinct orders, signed directed
coupling and intrinsic dynamics; it is not covered by ordinary consensus theorems.

## 20–31. Formation control

Third-order displacement formation applies the integrator-chain law to `p-p_des`.
Relative geometry is fixed while translation is free. Moving leader formation
adds reference velocity/acceleration feedforward and pinning. Containment fixes
leaders and applies `pdot_f=-k(L_ff p_f+L_fl p_l)`. The constructor requires a
positive-stable follower block, equivalent here to every follower being reachable
from some leader through positive-weight edges. The equilibrium lies in the
leaders' convex hull.

Distance formation descends `k/2 sum_e (||p_i-p_j||-d_e)^2`, with a denominator
floor near coincident nodes. Translation and rotation are free. Rigidity is needed
for locally unique shape; undesired equilibria prevent a global convergence claim.
Henneberg I/II moves produce generically minimally rigid planar graphs, but a
particular collinear or otherwise degenerate embedding can still be flexible.

Bearing control uses current unit direction g, target g*, and `P_g=I-gg^T`.
The range-aware negative gradient includes `P_g g*/distance`; the bearing-only
law omits the distance factor. Pair antisymmetry and orthogonality preserve
centroid and centered scale in continuous time. Coincident endpoints are rejected.
The [bearing-rigidity literature](https://arxiv.org/abs/1408.6552) explains the
necessary shape assumptions; arbitrary starting configurations are not guaranteed.

Pinned bearing control and localization use a block Laplacian assembled from
`I-g* g*^T`. The follower block must be positive definite; two anchors alone are
insufficient. These projectors constrain bearing lines and do not distinguish
opposite directions by themselves. Consistent anchor geometry selects the intended
realization. PI maneuvering uses `pdot_f=-kp(Bp)_f-ki z_f`, `zdot_f=(Bp)_f`;
double-integrator maneuvering uses `pddot_f=-(B(kp p+kv v))_f`. The prescribed
constant leader velocities must be compatible with the fixed bearing pattern.
See [Zhao and Zelazo's maneuvering formulation](https://arxiv.org/abs/1504.03517).

The SE(2) example descends bidirectional local-frame bearing error in position
and heading. It assumes fully actuated planar translation and rotation, plus
relative-heading information; it is not a unicycle controller. Translation,
positive scale and simultaneous world/heading rotation are free. Finite-time
displacement formation applies componentwise sign-power feedback to `p-p_des`.

## 32–34. Coverage

Grid samples approximate density-weighted Voronoi cells. Each agent follows
`pdot_i=k(c_i-p_i)` toward its cell's weighted centroid; empty cells stay put.
For fixed density and unlimited sensing this is a positive-scaled descent of the
sampled squared-distance objective. This discretizes the centroidal construction
in [Cortes, Martinez, Karatas and Bullo](https://citeseerx.ist.psu.edu/document?doi=5f67cca21a0914e2cf50d1a4d9b3044f91aecc1a&repid=rep1&type=pdf).

With finite sensing radius R, the reported objective is the weighted average of
`min(nearest_distance, R)^2`. Uncovered samples contribute R squared; they do not
vanish from the cost. Cell boundaries and sensing cutoffs make the discrete
objective piecewise smooth. The spatial example uses a volumetric grid. Adaptive
coverage means a prescribed moving density hotspot, not parameter learning.
Density is evaluated explicitly at time t. Its changing objective need not decrease,
and initial/final costs refer to different density fields.

## 35–43. Swarming and flocking

Swarming combines Morse-type attraction/repulsion with viscous damping. Boids
combines unit-direction separation, neighborhood velocity/position averages and
goal PD; self is included in the averages. Neither gives a general safety proof.

Free-space Olfati-Saber examples use sigma-distance pair potentials, smooth
finite-range weights and relative-velocity alignment. Algorithm 1 has no leader
and preserves collective momentum, but disconnected groups may fragment.
Algorithm 2 adds leader navigation; the archive variant saturates coordinates
individually. Its force parameters require `0<A<=B` to place the force zero at
the requested spacing. Accelerating references have no acceleration feedforward,
so exact trajectory tracking is not promised.

The examples titled Algorithm 3 preserve a practical archive variant: shifted
repulsive pair forces, exponential obstacle normals and velocity-projected or
fixed planar tangents. They do not implement the paper's projected beta-agent
law. The bounded simulator clips positions and caps speeds, then stops at majority
arrival; this is a hybrid heuristic, not settled agreement of every robot. The
Verlet-style option is not a classical conservative Verlet method because force
depends on velocity. These distinctions matter when applying the assumptions in
[Olfati-Saber's original paper](https://hal.elte.hu/~vicsek/downloads/papers/flocking_tac06-engineering.pdf).

## 44. Multi-agent navigation

Four robots have individual goals and second-order dynamics. Their clipped
accelerations combine goal attraction, damping, spherical-obstacle repulsion and
pairwise inter-agent repulsion. Each robot's motion therefore depends on the
positions of other robots. Finite forces, coincident agents and actuator clipping
prevent a universal collision-avoidance claim; sampled trajectories do not certify
continuous-time clearance.

## 45–52. Localization and estimation

Relative localization minimizes `0.5 sum ||p_j-p_i-z_ij||^2` with a fixed anchor.
Connected unit-weight proximal estimation uses a local regularized Jacobi update
of the same normal equations. Inconsistent measurements yield least-squares
estimates rather than exact physical coordinates.

Range reconstruction uses residual `r_e=(||p_i-p_j||^2-d_e^2)/2`, Jacobian R,
and gradient `R^T r`. Three noncollinear known planar anchors fix the frame but
do not alone make every graph uniquely localizable. Gauss–Newton solves damped
normal equations by relaxed block Jacobi with a coordinated global line search;
it is not fully decentralized. Both methods are local nonconvex procedures.

The PI average estimator implements
`xdot=-gamma x-kp Lx+ki L^T w+gamma u`, `wdot=-ki Lx`.
Its mean obeys `mean(x)'=gamma(mean(u)-mean(x))`; constant inputs converge, while
changing inputs generally produce lag. See [Freeman, Yang and Lynch](https://robotics.northwestern.edu/research/publications/stability-and-convergence-properties-of-dynamic-average-consensus-estimators.html).

Local observer banks exchange network estimates and pin each observer's own
signal. Exact moving-signal tracking needs the supplied common drift. The rigidity
estimator uses deflated shifted power iteration on `R^T R`; finite Metropolis
gossip approximates gauge removal, normalization and Rayleigh quotients. It uses
a fixed known planar framework and a global matrix-norm step bound, so it is not
an entirely local implementation of a changing-framework estimator. Convergence
also needs an initial component in the sought eigenspace and an appropriate gap.

## 53–60. Robustness and synchronization

Connectivity control descends a soft target cost plus
`beta sum r_e^2/(R^2-r_e^2)`, starting strictly inside each range boundary.
Rigidity control uses `beta/(lambda_4(R^T R)-lambda_min)`. Its eigenvalue gradient
assumes a simple eigenvalue; eigenvalue crossings can make it nonsmooth. Both
compute a compromise with their target, and neither barrier prevents collisions.
The rigidity eigenpair is centralized.

For an undirected positive base graph, effective resistance is `b^T L^+ b`.
A single negative edge of magnitude d retains strict disagreement stability if
`d R_eff < 1`. For multiple edges the coupled condition is
`I-sqrt(D) E^T L^+ E sqrt(D) > 0`. The objective decoder `2 tanh(theta)` illustrates
bounded sensitivity; it provides no cryptographic security.

Periodic event checking broadcasts when the current state differs sufficiently
from its held value. The dynamics use current self state and held neighbor states;
there is no arithmetic-mean invariant in general. A positive threshold floor
allows practical disagreement rather than requiring exact agreement. Event flags
are stored at the evaluation time, including initial broadcasts.

Riccati synchronization uses stabilizing CARE feedback `K=B^T P` and coupling
`c>1/(2 lambda_2)` on a connected undirected positive-weight graph. The modal
Lyapunov derivative is negative under this bound. Stabilizability of `(A,B)` is
required; this nominal state-feedback design is not robust H-infinity control.

Noisy consensus projects noise off the mean mode. Its stationary covariance is
`C=0.5 L^+`, obtained from `LC+CL=I-11^T/n`. Exact sampled Ornstein–Uhlenbeck
increments avoid Euler bias; `trace(C)` describes expected disagreement energy,
not convergence of an individual noisy trajectory. Field-of-view relay control
handles two noncoincident targets with a fixed camera axis and prescribed motion.
Gain and target-speed feasibility matter; a general moving-camera visibility
problem is outside this implementation.

## 61–65. Optimization and online control

Projection Newton uses exact sampled double integrators and a quadratic
formation/centroid/velocity objective on an undirected graph. Backward Riccati
recursion solves its LQ subproblem, followed by feedback projection and Armijo
backtracking. This is the LQ specialization of [Hauser's projection approach](https://www.sciencedirect.com/science/article/pii/S1474667015387334), not nonlinear PRONTO.

Optimal formation tracking instead uses centralized direct shooting with a
nonconvex squared-distance objective, exact discrete adjoints and L-BFGS-B.
Convergence means a local numerical solution; actuator and obstacle constraints
are absent. Controls have one fewer sample than state histories.

Preference agreement solves local quadratic integrator plans with terminal
agreement multipliers updated through the Laplacian. Positive state/input costs
make the local problems strictly convex. Finite dual iterations retain residuals.
Shrinking-horizon control reuses multipliers and executes one action after a fixed
communication budget, so terminal agreement need not be exact. Its unit time step
is part of the discrete model.

Online gradient formation uses distance gradients, their time derivatives,
centroid estimates and velocity alignment. Observer banks receive acceleration
information only from their closed neighborhoods. It is a local feedback design,
not an online solution of the finite-horizon optimization problem, and the coupled
nonlinear observer/controller has no global guarantee here.
