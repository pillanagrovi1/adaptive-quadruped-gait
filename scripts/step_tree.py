#!/usr/bin/env python3
"""Dump the reference STEP assembly as a flat list of parts with world placement.

Reads ~/reference/quadruped/Full assembly_step_/quadrupedKNGmain assy.STEP (not in the repo)
and writes build/reference/step_parts.json: one entry per part instance with its name,
the path of assembly names above it, its world bounding box (mm) and its solid volume (mm^3).
Every number in docs/robot.md that says "from the STEP" comes from this file.

    python3 scripts/step_tree.py            # ~1-3 min for the 59 MB file
"""
import json
import sys
from pathlib import Path

from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDF import TDF_Label
from OCP.TDocStd import TDocStd_Document
from OCP.TopLoc import TopLoc_Location
from OCP.XCAFDoc import XCAFDoc_DocumentTool

ROOT = Path(__file__).resolve().parents[1]
REF = Path.home() / "reference" / "quadruped"
STEP = REF / "Full assembly_step_" / "quadrupedKNGmain assy.STEP"
OUT = ROOT / "build" / "reference" / "step_parts.json"


def name_of(label) -> str:
    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return attr.Get().ToExtString()
    return "?"


def main():
    if not STEP.exists():
        sys.exit(f"{STEP} not found: unzip the reference design to {REF}")
    doc = TDocStd_Document(TCollection_ExtendedString("doc"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(str(STEP)) != IFSelect_RetDone:
        sys.exit("STEP read failed")
    reader.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = TDF_LabelSequence()
    st.GetFreeShapes(roots)
    parts = []

    def walk(label, loc: TopLoc_Location, path):
        ref = label
        if st.IsReference_s(label):
            ref_lab = TDF_Label()
            st.GetReferredShape_s(label, ref_lab)
            ref = ref_lab
        here = loc * st.GetLocation_s(label) if st.IsReference_s(label) else loc
        nm = name_of(label) if st.IsReference_s(label) else name_of(ref)
        if st.IsAssembly_s(ref):
            kids = TDF_LabelSequence()
            st.GetComponents_s(ref, kids)
            for i in range(1, kids.Length() + 1):
                walk(kids.Value(i), here, path + [nm])
            return
        shape = st.GetShape_s(ref).Moved(here)
        box = Bnd_Box()
        BRepBndLib.Add_s(shape, box)
        p0, p1 = box.CornerMin(), box.CornerMax()
        x0, y0, z0, x1, y1, z1 = p0.X(), p0.Y(), p0.Z(), p1.X(), p1.Y(), p1.Z()
        props = GProp_GProps()
        BRepGProp.VolumeProperties_s(shape, props)
        c = props.CentreOfMass()
        parts.append({"name": nm, "def": name_of(ref), "path": path,
                      "bbox": [x0, y0, z0, x1, y1, z1], "volume_mm3": props.Mass(),
                      "centroid": [c.X(), c.Y(), c.Z()]})

    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), [])
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(parts, indent=1))
    print(f"{len(parts)} part instances -> {OUT.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
