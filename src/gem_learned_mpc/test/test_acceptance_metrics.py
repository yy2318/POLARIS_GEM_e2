from gem_learned_mpc.validation import calculate_metrics,evaluate
rows=[]
for i in range(100): rows.append({'timestamp':i*.1,'x':i*.1,'y':0,'yaw':0,'speed':3,'nearest_path_index':i,'reference_x':i*.1,'reference_y':0,'reference_yaw':0,'reference_speed':3,'cte':.2,'heading_error':0,'speed_error':0,'steering_command':0,'acceleration_command':0,'solver_status':'SOLVED','solver_time_ms':20,'iteration_count':5,'state_valid':'true','fallback_active':'false'})
m=calculate_metrics(rows);cfg={'speed_limit_mps':5.5556,'speed_tolerance_mps':.05,'samples_within_cte_pct':95,'transient_cte_max_m':1.5,'cte_rmse_max_m':.5,'path_completion_min_pct':95,'solver_success_min_pct':99,'solver_p95_time_max_ms':100};c,p=evaluate(m,cfg);assert p and all(c.values())
print('PASS acceptance metrics')
