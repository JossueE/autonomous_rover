# Odom Comparison Results: yaw_sweep_90

## Reference

- Physical reference x: 0.000 m
- Physical reference y: 0.000 m
- Physical reference yaw: 0.00 deg
- Notes: From zero, turn 90 deg left, return center, turn 90 deg right, return center.

## Summary

| Source | Distance error (m) | Yaw error (deg) | Path length (m) | Frequency (Hz) |
| --- | ---: | ---: | ---: | ---: |
| Wheel odom | 0.0026 | -0.424 | 0.0647 | 50.01 |
| RTAB-Map odom | 0.0055 | 0.917 | 0.3274 | 3.89 |

## Cross-Odometry Agreement

- Paired samples: 333
- Mean XY disagreement: 0.0477 m
- Max XY disagreement: 0.0806 m
- Final XY disagreement: 0.0081 m
- Final yaw disagreement: -1.342 deg
- RTAB-Map lost percentage: 0.00 %

## Interpretation Template

Wheel odometry is expected to be repeatable on short straight segments, but it
accumulates heading and closure error when wheel slip or radius mismatch appears.
RTAB-Map odometry can reduce accumulated drift when the RGB-D/ICP registration is
well constrained, but it can degrade under poor lighting, low texture, sparse
geometry, motion blur, or aggressive angular velocity. These results use physical
marks as final-state references, so they support final error and closure claims,
not continuous ground-truth trajectory claims.
