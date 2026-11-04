import numpy as np
def volume(tris):
    return np.sum(np.einsum('ij,ij->i', tris[:,0], np.cross(tris[:,1], tris[:,2])))/6
