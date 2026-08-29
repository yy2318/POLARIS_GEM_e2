#!/usr/bin/env python3
import ast
from pathlib import Path
source=Path(__file__).parents[1]/'scripts'/'learned_mpc_node.py'
text=source.read_text()
tree=ast.parse(text)
required={'scenario_speed_callback','fault_callback','fault_type_callback','state_is_fresh','build_reference','solve_or_fallback','apply_hard_limits','publish_state_timeout_stop'}
found={n.name for n in ast.walk(tree) if isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef))}
missing=required-found
assert not missing, 'missing functions: '+str(missing)
for token in ['/gem_learned_mpc/scenario_target_speed','/gem_learned_mpc/fault_active','INJECTED_FAILURE','STATE_TIMEOUT','FAULT_RECOVERY_HOLD']:
 assert token in text, token
print('PASS: MPC interface static logic')
