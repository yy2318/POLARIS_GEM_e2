# MPC Test and Acceptance Report

Overall result: **FAIL**

## Metrics
- sample_count: 1019
- cte_rmse_m: 0.507198637948459
- cte_mae_m: 0.5071828160280372
- cte_max_m: 0.5140860924185766
- cte_p95_m: 0.5133977011357719
- samples_within_1m_pct: 100.0
- max_speed_mps: 0.00011037144400527992
- mean_solver_time_ms: 0.0
- p95_solver_time_ms: 0.0
- solver_success_pct: 0.0
- solver_failure_count: 1019
- path_completion_pct: 100.0
- nan_or_inf_count: 0
- fallback_count: 1019
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