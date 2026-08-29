# MPC Test and Acceptance Report

Overall result: **FAIL**

## Metrics
- sample_count: 1032
- cte_rmse_m: 1.3488500444862322e-05
- cte_mae_m: 1.1772609205124408e-05
- cte_max_m: 2.3281311536611378e-05
- cte_p95_m: 2.2107863806045153e-05
- samples_within_1m_pct: 100.0
- max_speed_mps: 0.00010146319395142102
- mean_solver_time_ms: 0.0
- p95_solver_time_ms: 0.0
- solver_success_pct: 0.0
- solver_failure_count: 1032
- path_completion_pct: 100.0
- nan_or_inf_count: 0
- fallback_count: 1032
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