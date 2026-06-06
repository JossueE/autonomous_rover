# Odom Comparison Results: straight_2m

## Reference

- Physical reference x: 2.000 m
- Physical reference y: 0.000 m
- Physical reference yaw: 0.00 deg
- Notes: Straight 2 m forward segment using floor marks as final reference.

## Summary

| Source | Distance error (m) | Yaw error (deg) | Path length (m) | Frequency (Hz) |
| --- | ---: | ---: | ---: | ---: |
| Wheel odom | 3.9478 | -4.056 | 1.9906 | 50.00 |
| RTAB-Map odom | 0.1220 | 4.538 | 2.0475 | 3.36 |

## Cross-Odometry Agreement

- Paired samples: 140
- Mean XY disagreement: 2.0006 m
- Max XY disagreement: 3.9963 m
- Final XY disagreement: 3.9904 m
- Final yaw disagreement: -8.328 deg
- RTAB-Map lost percentage: 0.00 %

## Interpretation Template

Wheel odometry is expected to be repeatable on short straight segments, but it
accumulates heading and closure error when wheel slip or radius mismatch appears.
RTAB-Map odometry can reduce accumulated drift when the RGB-D/ICP registration is
well constrained, but it can degrade under poor lighting, low texture, sparse
geometry, motion blur, or aggressive angular velocity. These results use physical
marks as final-state references, so they support final error and closure claims,
not continuous ground-truth trajectory claims.
