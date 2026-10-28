# Three-Dimensional Coverage

Three-dimensional centroidal coverage places agents throughout a volume rather than on a plane. The agents share responsibility for a spatial sensing task through nearest-agent partitions.

## Mathematical formulation

Each volumetric sample belongs to its nearest agent. With sample weights $\rho_m\ge0$, an agent’s centroid is

$$c_i=\frac{\sum_{m\in V_i}\rho_mq_m}{\sum_{m\in V_i}\rho_m},\qquad \dot p_i=k(c_i-p_i).$$

The objective is the density-weighted average squared distance from each sample to its nearest agent.

## Collective behavior

Agent motion changes the partition, so every centroid depends indirectly on all agent positions. A centroidal configuration balances the competing responsibility regions. For a fixed partition, the centroid minimizes its weighted squared-distance cost; repartitioning and centroid motion seek a locally efficient allocation of sensing resources.

## Assumptions and limitations

The density is fixed and nonnegative, and the volumetric grid approximates a continuous domain. Empty cells produce no displacement. Grid resolution affects both the centroids and the apparent final coverage quality. The objective is nonconvex in the full collection of positions, so global optimality is not guaranteed. Free spatial velocity actuation is assumed.
