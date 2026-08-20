#!/usr/bin/env python3
import argparse,json
from pathlib import Path
import numpy as np
import joblib
from sklearn.linear_model import Ridge
from sklearn.metrics import mean_squared_error
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline

def main():
    ap=argparse.ArgumentParser();ap.add_argument('dataset');ap.add_argument('--model',choices=['ridge','mlp'],default='ridge');ap.add_argument('--output-dir',required=True);ap.add_argument('--epochs',type=int,default=200);a=ap.parse_args()
    out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=True);z=np.load(a.dataset);X,Y,split=z['X'],z['Y'],z['split'];tr=split==0;va=split==1
    if a.model=='ridge':
        model=Pipeline([('scale',StandardScaler()),('ridge',Ridge(alpha=1.0))]);model.fit(X[tr],Y[tr]);joblib.dump(model,out/'dynamics_ridge.joblib');pred=model.predict(X[va] if va.any() else X[tr]);truth=Y[va] if va.any() else Y[tr]
    else:
        import torch
        torch.manual_seed(42);xm,xs,ym,ys=[z[k].astype(np.float32) for k in ['xm','xs','ym','ys']]
        xn=((X-xm)/xs).astype(np.float32);yn=((Y-ym)/ys).astype(np.float32)
        net=torch.nn.Sequential(torch.nn.Linear(X.shape[1],64),torch.nn.Tanh(),torch.nn.Linear(64,64),torch.nn.Tanh(),torch.nn.Linear(64,Y.shape[1]))
        opt=torch.optim.Adam(net.parameters(),lr=1e-3,weight_decay=1e-5);lossfn=torch.nn.MSELoss();xt=torch.from_numpy(xn[tr]);yt=torch.from_numpy(yn[tr])
        for epoch in range(a.epochs):
            opt.zero_grad();loss=lossfn(net(xt),yt);loss.backward();opt.step()
            if epoch%25==0:print(epoch,float(loss))
        net.eval();example=torch.zeros(X.shape[1]);torch.jit.trace(net,example).save(str(out/'dynamics_model.pt'));np.savez(out/'dynamics_model.norm.npz',xm=xm,xs=xs,ym=ym,ys=ys)
        idx=va if va.any() else tr
        with torch.no_grad():pred=net(torch.from_numpy(xn[idx])).numpy()*ys+ym
        truth=Y[idx]
    rmse=np.sqrt(np.mean((pred-truth)**2,axis=0));names=['vx','vy','yaw_rate','steering_angle'];metrics=dict(zip(names,rmse.tolist()));(out/'metrics.json').write_text(json.dumps(metrics,indent=2));np.savez_compressed(out/'validation_predictions.npz',truth=truth,prediction=pred)
    print('RMSE',metrics)
if __name__=='__main__':main()
