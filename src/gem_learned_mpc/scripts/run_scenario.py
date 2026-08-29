#!/usr/bin/env python3
import argparse,csv,os,signal,subprocess,sys,time,yaml
from pathlib import Path

def run(cmd,env=None): return subprocess.Popen(cmd,env=env,preexec_fn=os.setsid)
def stop(p):
 if p and p.poll() is None:
  os.killpg(os.getpgid(p.pid),signal.SIGINT)
  try:p.wait(timeout=10)
  except subprocess.TimeoutExpired:os.killpg(os.getpgid(p.pid),signal.SIGKILL)
def main():
 p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--scenario',required=True);p.add_argument('--launch-mpc',action='store_true');p.add_argument('--model-json',default='');a=p.parse_args();cfg=yaml.safe_load(open(a.config));c=cfg['common'];s=cfg['scenarios'][a.scenario];root=Path(c['results_root']);root.mkdir(parents=True,exist_ok=True);csv_path=root/s['output_csv'];procs=[]
 try:
  subprocess.check_call(['rosrun','gem_learned_mpc','set_scenario_initial_pose.py','--config',a.config,'--scenario',a.scenario]);time.sleep(float(c.get('settle_time_s',2)))
  if a.launch_mpc:procs.append(run(['roslaunch','gem_learned_mpc','full_demo.launch','waypoint_file:='+c['waypoint_file'],'vehicle_model_name:='+c['vehicle_model_name'],'model_json:='+a.model_json]))
  procs.append(run(['rosrun','gem_learned_mpc','scenario_speed_override.py','_target_speed_mps:='+str(s['target_speed_mps'])]))
  procs.append(run(['roslaunch','gem_learned_mpc','acceptance_monitor.launch','output_csv:='+str(csv_path)]))
  if a.scenario=='solver_fault':procs.append(run(['rosrun','gem_learned_mpc','fault_injector.py','_fault_type:='+s['fault_type'],'_fault_start_s:='+str(s['fault_start_s']),'_fault_duration_s:='+str(s['fault_duration_s'])]))
  deadline=time.time()+float(c['test_duration_s']);print('Running',a.scenario,'output',csv_path)
  while time.time()<deadline:time.sleep(1)
 finally:
  for q in reversed(procs):stop(q)
 print('Scenario completed:',a.scenario)
if __name__=='__main__':main()
