"""Regenerate everything:  python build.py
Needs only numpy, scipy-free; Pillow for the PNG renders.  Edit params.py first."""
import subprocess, sys
for s in ["export_stl.py", "parts_sheet.py", "render_all.py", "export_viewer.py"]:
    print("==>", s); subprocess.check_call([sys.executable, s])
print("Optional collision check:  python clearance.py")
