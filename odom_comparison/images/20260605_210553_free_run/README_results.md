# Odom Comparison Results: free_run

## Reference

- Physical reference x: 0.000 m
- Physical reference y: 0.000 m
- Physical reference yaw: 0.00 deg
- Notes: Free trajectory recorded until the operator stops the trial with Ctrl+C.

## Summary

| Source | Distance error (m) | Yaw error (deg) | Path length (m) | Frequency (Hz) |
| --- | ---: | ---: | ---: | ---: |
| Wheel odom | 0.2760 | 4.824 | 15.4897 | 49.98 |
| RTAB-Map odom | 0.1018 | -3.166 | 16.5964 | 3.85 |

## Cross-Odometry Agreement

- Paired samples: 2241
- Mean XY disagreement: 3.1835 m
- Max XY disagreement: 8.4293 m
- Final XY disagreement: 0.3709 m
- Final yaw disagreement: 7.991 deg
- RTAB-Map lost percentage: 0.00 %

## Interpretation Template

Wheel odometry is expected to be repeatable on short straight segments, but it
accumulates heading and closure error when wheel slip or radius mismatch appears.
RTAB-Map odometry can reduce accumulated drift when the RGB-D/ICP registration is
well constrained, but it can degrade under poor lighting, low texture, sparse
geometry, motion blur, or aggressive angular velocity. These results use physical
marks as final-state references, so they support final error and closure claims,
not continuous ground-truth trajectory claims.
