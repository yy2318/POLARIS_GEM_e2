#!/usr/bin/env python3
import argparse,json,numpy as np
p=argparse.ArgumentParser();p.add_argument('dataset');p.add_argument('--output-dir',required=True);p.add_argument('--epochs',type=int,default=300);a=p.parse_args();import torch,torch.nn as nn
d=np.load(a.dataset);X=d['x'].astype('float32');Y=d['y'].astype('float32');im=X.mean(0);isd=X.std(0)+1e-6;om=Y.mean(0);osd=Y.std(0)+1e-6;xt=torch.tensor((X-im)/isd);yt=torch.tensor((Y-om)/osd);m=nn.Sequential(nn.Linear(7,64),nn.Tanh(),nn.Linear(64,64),nn.Tanh(),nn.Linear(64,5));opt=torch.optim.Adam(m.parameters(),1e-3)
for e in range(a.epochs):opt.zero_grad();loss=((m(xt)-yt)**2).mean();loss.backward();opt.step()
from pathlib import Path;o=Path(a.output_dir);o.mkdir(parents=True,exist_ok=True);torch.save(m.state_dict(),o/'model.pt');layers=[]
for v in [m[0],m[2],m[4]]:layers.append({'weight':v.weight.detach().numpy().tolist(),'bias':v.bias.detach().numpy().tolist()})
json.dump({'dt':.1,'input_mean':im.tolist(),'input_std':isd.tolist(),'output_mean':om.tolist(),'output_std':osd.tolist(),'layers':layers},open(o/'model.json','w'));print('loss',float(loss))
