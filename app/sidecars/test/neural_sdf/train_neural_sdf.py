#!/usr/bin/env python3
"""Small CUDA Neural SDF experiment for a PLY point cloud with normals."""
from __future__ import annotations
import argparse, json, struct
from pathlib import Path
import numpy as np
import torch
from torch import nn

TYPE = {"double":"<f8", "float":"<f4", "uchar":"u1", "uint8":"u1", "int":"<i4"}

def load_ply(path: Path):
    with path.open("rb") as f:
        header=[]
        while True:
            line=f.readline().decode("ascii").strip(); header.append(line)
            if line=="end_header": break
        if "format binary_little_endian 1.0" not in header: raise ValueError("binary little-endian PLY required")
        n=int(next(x.split()[2] for x in header if x.startswith("element vertex")))
        props=[(x.split()[1], x.split()[2]) for x in header if x.startswith("property")]
        dtype=np.dtype([(name,TYPE[kind]) for kind,name in props])
        data=np.fromfile(f,dtype=dtype,count=n)
    pts=np.c_[data["x"],data["y"],data["z"]].astype("float32")
    nrm=np.c_[data["nx"],data["ny"],data["nz"]].astype("float32")
    nrm/=np.maximum(np.linalg.norm(nrm,axis=1,keepdims=True),1e-8)
    return pts,nrm

class SDF(nn.Module):
    def __init__(self):
        super().__init__(); self.net=nn.Sequential(nn.Linear(3,128),nn.SiLU(),nn.Linear(128,128),nn.SiLU(),nn.Linear(128,128),nn.SiLU(),nn.Linear(128,1))
    def forward(self,x): return self.net(x)

def main():
    a=argparse.ArgumentParser(); a.add_argument("--input",type=Path,required=True); a.add_argument("--out",type=Path,required=True); a.add_argument("--device",default="cuda"); a.add_argument("--steps",type=int,default=5000); a.add_argument("--batch",type=int,default=8192); args=a.parse_args()
    if args.device.startswith("cuda") and not torch.cuda.is_available(): raise RuntimeError("CUDA requested but unavailable")
    pts,nrm=load_ply(args.input); center=pts.mean(0); scale=float(np.max(np.linalg.norm(pts-center,axis=1))); pts=(pts-center)/scale
    dev=torch.device(args.device); points=torch.from_numpy(pts).to(dev); normals=torch.from_numpy(nrm).to(dev)
    model=SDF().to(dev); opt=torch.optim.Adam(model.parameters(),lr=1e-3); noise=0.003
    for step in range(args.steps):
        ix=torch.randint(len(points),(args.batch,),device=dev); p=points[ix]; n=normals[ix]
        p.requires_grad_(True); surface=model(p); grad=torch.autograd.grad(surface,p,torch.ones_like(surface),create_graph=True)[0]
        q=p.detach()+n*torch.randn((args.batch,1),device=dev)*noise; target=((q-p.detach())*n).sum(-1,keepdim=True)
        off=model(q); eik=torch.rand_like(p)*2-1; eik.requires_grad_(True); g=torch.autograd.grad(model(eik),eik,torch.ones((args.batch,1),device=dev),create_graph=True)[0]
        loss=surface.abs().mean()+2*(off-target).abs().mean()+0.1*(1-(grad*n).sum(-1)).abs().mean()+0.05*(g.norm(dim=-1)-1).square().mean()
        opt.zero_grad(); loss.backward(); opt.step()
        if step%250==0: print(f"step={step} loss={loss.item():.6f}",flush=True)
    args.out.mkdir(parents=True,exist_ok=True); torch.save({"state_dict":model.state_dict(),"center":center,"scale":scale},args.out/"model.pt")
    (args.out/"normalization.json").write_text(json.dumps({"center":center.tolist(),"scale":scale},indent=2))
    (args.out/"train_metrics.json").write_text(json.dumps({"steps":args.steps,"final_loss":float(loss.item()),"points":int(len(pts))},indent=2))
if __name__=="__main__": main()