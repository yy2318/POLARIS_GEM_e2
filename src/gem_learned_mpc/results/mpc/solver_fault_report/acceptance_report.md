# MPC Test and Acceptance Report

Overall result: **FAIL**

## Metrics
- sample_count: 1034
- cte_rmse_m: 1.3455690152544757e-05
- cte_mae_m: 1.1726837623342438e-05
- cte_max_m: 2.326104560545326e-05
- cte_p95_m: 2.2084758355322004e-05
- samples_within_1m_pct: 100.0
- max_speed_mps: 0.0001313109967122902
- mean_solver_time_ms: 0.0
- p95_solver_time_ms: 0.0
- solver_success_pct: 0.0
- solver_failure_count: 1034
- path_completion_pct: 100.0
- nan_or_inf_count: 0
- fallback_count: 1034
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