# MPC Test and Acceptance Report

Overall result: **FAIL**

## Metrics
- sample_count: 959
- cte_rmse_m: 1.2498510040179479e-05
- cte_mae_m: 1.0912760912434328e-05
- cte_max_m: 2.155309819612354e-05
- cte_p95_m: 2.0469937834229058e-05
- samples_within_1m_pct: 100.0
- max_speed_mps: 0.0001030931133444936
- mean_solver_time_ms: 0.0
- p95_solver_time_ms: 0.0
- solver_success_pct: 0.0
- solver_failure_count: 959
- path_completion_pct: 100.0
- nan_or_inf_count: 0
- fallback_count: 959
- speed_limit_mps: 5.5556

## Checks
- [x] max_speed
- [x] cte_95pct
- [x] transient_cte
- [x] cte_rmse
- [x] completion
- [ ] solver_success
- [x] solver_time
- [x] finite