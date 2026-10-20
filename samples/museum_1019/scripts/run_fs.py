import sys, time, numpy as np
from viz import load
from freespace import scan_rays, search
from register import walls, icp_planar
import open3d as o3d
mode=sys.argv[1]
d4,d5=load('room04'),load('room05')
T45=np.load('T_04_05.npy')
if mode=='test':
    src=scan_rays(d5); dst=scan_rays(d4); yaw_step=2.0
else:
    d1=load('room01'); src=scan_rays(d1); dst=scan_rays(d4)+scan_rays(d5,T45); yaw_step=1.0
t=time.time()
c,_=search(src,dst,yaw_step=yaw_step,top=12,lam=float(sys.argv[2]) if len(sys.argv)>2 else 1.0)
print('search',round(time.time()-t),'s')
for sc,yd,T in c:
    print(f'score={sc:.3f} yaw={np.degrees(np.arctan2(T[1,0],T[0,0])):.1f} t={T[:2,3].round(2)}')
np.save(f'fs_{mode}.npy',np.array([T for _,_,T in c])); np.save(f'fs_{mode}_sc.npy',np.array([s for s,_,_ in c]))
