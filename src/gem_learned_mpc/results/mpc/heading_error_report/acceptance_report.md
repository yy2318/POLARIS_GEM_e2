# MPC Test and Acceptance Report

Overall result: **FAIL**

## Metrics
- sample_count: 1030
- cte_rmse_m: 0.008294528595410423
- cte_mae_m: 0.007246486394964086
- cte_max_m: 0.014201216852800484
- cte_p95_m: 0.013507184874490874
- samples_within_1m_pct: 100.0
- max_speed_mps: 0.00010146178398227069
- mean_solver_time_ms: 0.0
- p95_solver_time_ms: 0.0
- solver_success_pct: 0.0
- solver_failure_count: 1030
- path_completion_pct: 100.0
- nan_or_inf_count: 0
- fallback_count: 1030
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