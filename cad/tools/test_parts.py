import sys, os, time, struct, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "brep"))
import racklift_brep as R
STL = os.path.join(os.path.dirname(__file__), "..", "..", "archive", "v2_original_stl")
def stl_vol(p):
    d = open(p, "rb").read(); n = struct.unpack("<I", d[80:84])[0]
    a = np.frombuffer(d[84:84+50*n], dtype=[("n","<f4",3),("v","<f4",9),("a","<u2")])["v"].reshape(-1,3,3).astype(float)
    return np.einsum('ij,ij->i', a[:,0], np.cross(a[:,1], a[:,2])).sum()/6
only = sys.argv[1:]
for name, (fn, M, q, g) in R.PRINTED.items():
    if only and not any(o in name for o in only): continue
    t = time.time()
    try:
        s = fn(); sh = s.shape
        ns = len(sh.solids()); v = sh.volume; ok = sh.is_valid
        ref = stl_vol(os.path.join(STL, name + ".stl"))
        print(f"{name:28s} solids={ns} valid={ok} vol={v:9.1f} stl={ref:9.1f} diff={100*(v-ref)/ref:+6.2f}%  faces={len(sh.faces())} {time.time()-t:.1f}s", flush=True)
    except Exception as e:
        import traceback; print(name, "FAILED", repr(e)); traceback.print_exc()
