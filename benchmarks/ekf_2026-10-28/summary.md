| 모드 | 성공/전체 | 위치 RMSE (m) | 방위 RMSE (°) | 표본 (Hz) | 표본 비율 | p95 추정 지연 (s) | TF 공백 (s) | 실제 시간 (s) | RTF |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| baseline | 3/3 | 0.000067 | 0.010 | 7.781 | 0.776 | 0.031 | 0.034 | 40.695 | 0.996 |
| ekf_overlay | 3/3 | 0.000154 | 0.013 | 7.786 | 0.779 | 0.103 | 0.034 | 41.597 | 0.996 |
| ekf_independent | 3/3 | 0.002592 | 0.119 | 7.161 | 0.716 | 0.104 | 0.105 | 41.075 | 0.997 |

모의 시험 결론: **baseline** — Independent EKF did not meet the prespecified improvement criterion in these simulated trials

Gazebo diff-drive odometry may be world-derived. A simulation winner is not validated for the real TurtleBot3; repeat with an external reference and safe physical course.

동일 overlay 시험에서 원본 /odom 위치 RMSE: 0.000072 m; EKF 후보 /odometry/filtered 위치 RMSE: 0.000154 m.
