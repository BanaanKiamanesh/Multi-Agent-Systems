# Field-of-View Relay Control

Field-of-view relay control moves one relay agent so that two moving agents remain inside its viewing cone. The relay responds to their bearings, while the two targets follow prescribed trajectories.

## Mathematical formulation

Let a be the fixed unit camera axis, $g_i$ the relay-to-target bearing and $P_{g_i}=I-g_ig_i^T$. If both targets lie on the same side of the axis, the relay uses the projected-axis direction of the target with the largest angular deviation. Otherwise it combines both projected directions. The relay velocity is the negative of that selected projection sum, scaled by a positive gain.

## Collective behavior

Moving the relay changes both viewing bearings without rotating the camera axis. The switching rule chooses the target most at risk on one side or balances targets on opposite sides. This is one-way multi-agent coupling: target motion drives the relay response, while the relay does not command the targets.

## Assumptions and limitations

Exactly two noncoincident targets, an acute cone half-angle and initially valid visibility are assumed. Feasibility depends on target speeds and relay gain. The fixed-axis planar model does not solve arbitrary moving-camera visibility or occlusion problems. Sampled visibility checks cannot certify visibility between samples, especially near switching boundaries.
