#!/usr/bin/env python3
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

def main():
    ap=argparse.ArgumentParser();ap.add_argument('predictions');ap.add_argument('--out-dir',required=True);a=ap.parse_args();o=Path(a.out_dir);o.mkdir(parents=True,exist_ok=True)
    z=np.load(a.predictions);y=z['truth'];p=z['prediction'];names=['vx','vy','yaw_rate','steering_angle'];rmse=np.sqrt(np.mean((p-y)**2,axis=0))
    fig,ax=plt.subplots(2,2,figsize=(12,7))
    for i,a0 in enumerate(ax.ravel()):a0.plot(y[:1000,i],label='Measured');a0.plot(p[:1000,i],label='Predicted',alpha=.8);a0.set_title(f'{names[i]} RMSE={rmse[i]:.4f}');a0.grid(True);a0.legend()
    fig.tight_layout();fig.savefig(o/'input_output_prediction.png',dpi=180);plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,4));ax.bar(names,rmse);ax.set_ylabel('RMSE');ax.grid(True,axis='y');fig.tight_layout();fig.savefig(o/'rmse.png',dpi=180);plt.close(fig)
    (o/'rmse.json').write_text(json.dumps(dict(zip(names,rmse.tolist())),indent=2));print(dict(zip(names,rmse)))
if __name__=='__main__':main()
