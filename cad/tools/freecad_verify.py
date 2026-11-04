"""Run with freecadcmd. Imports every STEP from models/, checks the solids, saves an .FCStd next to each assembly.
default folder = <repo>/models (override with env RL_OUT)"""
import os, glob
import FreeCAD as App
import Part, Import

OUT = os.environ.get("RL_OUT") or os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "models"))
bad = []
for f in sorted(glob.glob(os.path.join(OUT, "design", "*.step"))):
    s = Part.read(f)
    if not (s.isValid() and len(s.Solids) == 1):
        bad.append((os.path.basename(f), s.isValid(), len(s.Solids)))
print("design parts checked, problems:", bad)

for f in sorted(glob.glob(os.path.join(OUT, "assembly", "*.step"))):
    name = os.path.splitext(os.path.basename(f))[0]
    doc = App.newDocument(name)
    Import.insert(f, doc.Name)
    doc.recompute()
    feats = [o for o in doc.Objects if o.isDerivedFrom("Part::Feature") and o.Shape.Solids]
    solids = sum(len(o.Shape.Solids) for o in feats if o.TypeId == "Part::Feature" and not o.OutList)
    invalid = [o.Label for o in feats if not o.Shape.isValid()]
    doc.saveAs(os.path.join(OUT, "assembly", name + ".FCStd"))
    print(name, "objects:", len(doc.Objects), "leaf solids:", solids, "invalid:", invalid[:5])
    App.closeDocument(doc.Name)
print("VERIFY DONE")
