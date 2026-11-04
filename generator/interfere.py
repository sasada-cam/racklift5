import numpy as np
def bb(T): return T.reshape(-1,3).min(0), T.reshape(-1,3).max(0)
def voxelize(T, lo, n, h):
    nx, ny, nz = n
    off = np.array([0.0137, 0.0291])*h
    x = T[:,:,0]; y = T[:,:,1]
    hist = np.zeros((nx, ny, nz+1), np.int32)
    cx0 = lo[0]+h/2+off[0]; cy0 = lo[1]+h/2+off[1]
    i0 = np.clip(np.ceil((x.min(1)-cx0)/h).astype(int),0,nx); i1 = np.clip(np.floor((x.max(1)-cx0)/h).astype(int),-1,nx-1)
    j0 = np.clip(np.ceil((y.min(1)-cy0)/h).astype(int),0,ny); j1 = np.clip(np.floor((y.max(1)-cy0)/h).astype(int),-1,ny-1)
    for t in np.nonzero((i1>=i0)&(j1>=j0))[0]:
        xs = cx0 + h*np.arange(i0[t], i1[t]+1); ys = cy0 + h*np.arange(j0[t], j1[t]+1)
        gx, gy = np.meshgrid(xs, ys, indexing='ij')
        (ax,ay,az),(bx,by,bz),(qx,qy,qz) = T[t]
        den = (by-qy)*(ax-qx)+(qx-bx)*(ay-qy)
        if abs(den) < 1e-12: continue
        w0 = ((by-qy)*(gx-qx)+(qx-bx)*(gy-qy))/den; w1 = ((qy-ay)*(gx-qx)+(ax-qx)*(gy-qy))/den; w2 = 1-w0-w1
        m = (w0>=0)&(w1>=0)&(w2>=0)
        if not m.any(): continue
        z = w0*az+w1*bz+w2*qz
        k = np.clip(np.ceil((z-lo[2]-h/2)/h).astype(int), 0, nz)
        ii, jj = np.nonzero(m)
        np.add.at(hist, (ii+i0[t], jj+j0[t], k[m]), 1)
    return (np.cumsum(hist, axis=2)[:,:,:nz] % 2) == 1
def interference(TA, TB, h=1.0):
    a0,a1 = bb(TA); b0,b1 = bb(TB)
    lo = np.maximum(a0,b0); hi = np.minimum(a1,b1)
    if np.any(hi-lo <= 0): return 0.0, None
    # full voxelization needs the whole column extent in z for parity -> use full z of each but clip region
    n = np.maximum(np.ceil((hi-lo)/h).astype(int),1)
    # voxelize each solid on a grid covering overlap in x,y but full own z, then crop
    def vox(T):
        t0,t1 = bb(T)
        lo2 = np.array([lo[0],lo[1],t0[2]-h]); nz = int(np.ceil((t1[2]-t0[2]+2*h)/h))
        V = voxelize(T, lo2, (n[0],n[1],nz), h)
        k0 = int(np.floor((lo[2]-lo2[2])/h)); return V[:,:,k0:k0+n[2]]
    VA, VB = vox(TA), vox(TB)
    m = min(VA.shape[2], VB.shape[2])
    I = VA[:,:,:m] & VB[:,:,:m]
    c = int(I.sum())
    if c == 0: return 0.0, None
    idx = np.argwhere(I).mean(0)*h + lo
    return c*h**3, idx
