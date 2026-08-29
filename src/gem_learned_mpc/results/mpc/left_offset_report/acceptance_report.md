# MPC Test and Acceptance Report

Overall result: **FAIL**

## Metrics
- sample_count: 948
- cte_rmse_m: 0.500010704667272
- cte_mae_m: 0.5000107046310087
- cte_max_m: 0.5000212195419583
- cte_p95_m: 0.5000201493262771
- samples_within_1m_pct: 100.0
- max_speed_mps: 0.00016085849308846803
- mean_solver_time_ms: 0.0
- p95_solver_time_ms: 0.0
- solver_success_pct: 0.0
- solver_failure_count: 948
- path_completion_pct: 100.0
- nan_or_inf_count: 0
- fallback_count: 948
- speed_limit_mps: 5.5556

## Checks
- [x] max_speed
- [x] cte_95pct
- [x] transient_cte
- [ ] cte_rmse
- [x] completion
- [ ] solver_success
- [x] solver_time
- [x] finite