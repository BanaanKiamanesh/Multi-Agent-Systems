# Online Gradient Formation Tracking

Online gradient formation tracking coordinates a moving formation through feedback rather than solving a complete future trajectory optimization. It combines distance regulation with network estimates of collective position and velocity.

## Mathematical formulation

For edge residual $e_{ij}=\|p_i-p_j\|^2-d_{ij}^2$, the distance-gradient contribution is $e_{ij}(p_i-p_j)$. Its time derivative uses relative velocity. Agent acceleration combines negative distance gradient, gradient-rate damping, centroid-tracking terms and Laplacian velocity alignment.

Observer banks exchange network estimates, pin each agent’s own state and receive acceleration information only from closed neighborhoods.

## Collective behavior

The gradient term restores shape, its rate term anticipates changing edge error, and velocity alignment promotes coherent motion. Estimated centroids connect local information to a collective translation objective. The observers and physical agents evolve together, so tracking quality depends on both estimator convergence and control response.

## Assumptions and limitations

The distance objective is nonconvex and the coupled observer–controller has no global convergence guarantee here. Desired geometry, graph connectivity and observer bandwidth affect local behavior. The common reference velocity is prescribed. This feedback law does not produce the solution of a finite-horizon optimal-control problem and contains no independent collision or actuator constraints.
