#!/usr/bin/env python3
"""Reproduce P1/P2 CAD patterns, CMTracker/Motive assets and instructions.

Offline authoring only; never installed on the robot. Requires numpy; PDF output
also uses matplotlib and reportlab. No CMTracker recognition model is assumed.
Run from any directory: python generate_player_patterns.py --pdf
"""
from __future__ import annotations

import argparse
import copy
import csv
from decimal import Decimal
import hashlib
import io
import itertools
import json
import math
from pathlib import Path
import uuid
import xml.etree.ElementTree as ET

import numpy as np

SHELL = Path(__file__).resolve().parents[1]
REPO = SHELL.parents[1]
RUNTIME = REPO / "hope_ws/src/hope_bringup/config/pants_profiles"
TABLE = SHELL / "documents/marker_transforms_pelvis_link_stickers.csv"
P1_ORDER = "S19 S20 S11 S12 S22 S21 S13 S14 S04 S05 S16 S18".split()
FRONT = set("S11 S12 S13 S19 S20 S21 S22".split())
DATE = "2026-10-27"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def json_bytes(value):
    return (json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n").encode()


def geometry():
    with TABLE.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    names = [r["marker_id"] for r in rows]
    positions = np.array([[float(Decimal(r[k]) * 1000) for k in ("x_m", "y_m", "z_m")] for r in rows])
    normals = np.array([[float(r[k]) for k in ("normal_x", "normal_y", "normal_z")] for r in rows])
    return rows, names, positions, normals


def choose_pattern():
    """Exhaustive, lexicographic CAD criterion; not an all-view guarantee.

    First minimize intersection with fixed P1 (global lower bound seven).
    Among those 792 optima: minimize nonshared 4- then 3-point distance-signature
    matches within 2 mm; maximize minimum P2 facing count, P1-only angular
    coverage, views with >=6 P2 facing points, mean P2 count and views with >=2
    P1-only points. Lexicographically smallest station tuple breaks a final tie.
    Sorted edge signatures are a conservative screen, not a vendor fit solver.
    """
    _, names, p, normals = geometry()
    p1 = {names.index(name) for name in P1_ORDER}
    extra = set(range(len(names))) - p1
    distances = np.linalg.norm(p[:, None] - p[None, :], axis=2)
    screens = {}
    for k in (3, 4):
        subsets = np.array(list(itertools.combinations(range(17), k)))
        edges = list(itertools.combinations(range(k), 2))
        signatures = np.sort(np.stack([distances[subsets[:, i], subsets[:, j]] for i, j in edges], axis=1), axis=1)
        shared = np.array([set(s) <= p1 for s in subsets])
        error = np.max(abs(signatures[:, None] - signatures[shared][None]), axis=2).min(axis=1)
        masks = np.array([sum(1 << int(i) for i in s) for s in subsets])
        screens[k] = (masks, shared, error)
    azimuths = np.tile(np.arange(0, 360, 5), 4)
    elevations = np.repeat([0, 15, 30, 45], 72)
    az, el = np.deg2rad(azimuths), np.deg2rad(elevations)
    directions = np.array([np.cos(el)*np.cos(az), np.cos(el)*np.sin(az), np.sin(el)]).T
    facing = directions @ normals.T >= math.cos(math.radians(60))
    ranking = []
    for common in itertools.combinations(sorted(p1), 7):
        selected = sorted(extra | set(common))
        p1_only = sorted(p1 - set(common))
        mask = sum(1 << i for i in selected)
        nearby = {}
        for k, (masks, shared, error) in screens.items():
            included = ((masks & mask) == masks) & ~shared
            nearby[str(k)] = {str(t): int(np.sum(included & (error <= t))) for t in (0.000001, 2, 3, 5)}
        count = facing[:, selected].sum(axis=1)
        unique = facing[:, p1_only].sum(axis=1)
        # Sort ascending; all negative entries are maximization criteria.
        key = (nearby["4"]["2"], nearby["3"]["2"], -int(count.min()),
               -int(np.sum(unique >= 1)), -int(np.sum(count >= 6)),
               -int(count.sum()), -int(np.sum(unique >= 2)), tuple(selected))
        ranking.append((key, selected, nearby))
    ranking.sort(key=lambda item: item[0])
    key, selected, nearby = ranking[0]
    p2 = set(selected)
    def view_stats(which, cone=60):
        visibility = directions @ normals.T >= math.cos(math.radians(cone))
        count = visibility[:, sorted(which)].sum(axis=1)
        return {"minimum": int(count.min()), "mean": float(count.mean()),
                "views_at_least_one": int(np.sum(count >= 1)), "views_at_least_six": int(np.sum(count >= 6))}
    report = {
        "schema_version": 1, "date": DATE,
        "source_table_sha256": sha(TABLE), "units": "mm", "cad_stations": names,
        "p1_order": P1_ORDER, "p2_order": [names[i] for i in selected],
        "all_12_of_17_patterns": math.comb(17, 12), "minimum_shared_markers": 7,
        "maximum_separation_candidates_evaluated": len(ranking),
        "shared": [names[i] for i in sorted(p1 & p2)],
        "p1_only": [names[i] for i in sorted(p1 - p2)],
        "p2_only": [names[i] for i in sorted(p2 - p1)],
        "criterion_priority": ["minimize overlap", "minimize nonshared four-point signature candidates at 2 mm",
            "minimize nonshared three-point signature candidates at 2 mm", "maximize minimum P2 normal-facing count",
            "maximize views with a P1-only facing point", "maximize views with at least six P2 facing points",
            "maximize total P2 facing count", "maximize views with at least two P1-only facing points",
            "lexicographically smallest station tuple"],
        "signature_metric": "Minimum over P1 subsets of max absolute difference of sorted pairwise-distance arrays. A necessary, not sufficient, rigid-match condition; includes reflected signatures.",
        "design_distance_tolerance_mm": 2,
        "tolerance_status": "Engineering screening assumption, not a measured CMTracker tolerance or camera accuracy.",
        "nonshared_signature_candidates_by_tolerance_mm": nearby,
        "identical_shared_subsets": {str(k): math.comb(7, k) for k in range(3, 8)},
        "view_model": {"azimuth_degrees": list(range(0, 360, 5)), "elevation_degrees": [0, 15, 30, 45],
            "count": len(directions), "max_normal_incidence_degrees": 60,
            "includes_shell_or_torso_occlusion": False, "includes_camera_triangulation": False,
            "interpretation": "Normal-facing upper bound per direction, not visible/reconstructed marker counts."},
        "view_stats": {"P1": view_stats(p1), "P2": view_stats(p2),
            "P1_only": view_stats(p1-p2), "P2_only": view_stats(p2-p1)},
        "incidence_sensitivity": {str(c): {"P1": view_stats(p1,c), "P2": view_stats(p2,c),
            "P1_only": view_stats(p1-p2,c), "P2_only": view_stats(p2-p1,c)} for c in (45, 70)},
        "top_candidates": [{"p2": [names[i] for i in s], "ranking_key": list(k[:-1]),
            "near_signature_counts": near} for k, s, near in ranking[:5]],
        "guarantee": "NONE: shared-only subsets can be identical. All P2-only sites are rear sites. No pose-only VRPN signal certifies identity or geometry.",
        "minMarker": 6, "minMarker_note": "Inherited from the supplied format; six or seven shared markers can still be ambiguous. No new runtime threshold is imposed.",
    }
    return report


def write_assets(report):
    rows, names, _, _ = geometry()
    by_name = dict(zip(names, rows))
    directory = SHELL / "vrpn_asset"
    catalog = json.loads((RUNTIME / "profiles.json").read_text())
    spec = catalog["profiles"]["v4"]
    # The top-level geometry is now side-specific, rather than one shared array.
    spec.pop("marker_names", None)
    spec.pop("marker_positions_m", None)
    spec.update(label="V4 shell: distinct P1/P2 12-sticker patterns", marker_count=12, station_count=17)
    spec["source"]["description"] = "17-station v3 shell, side-specific 12-sticker subsets; optical surface 0.20 mm"
    manifest = {
        "format": "HOPE_CMTRACKER_ASSET_RELEASE", "schema_version": 2,
        "shell_design": "mocap_sticker_shell_v3", "runtime_profile": "v4",
        "body_frame": "pelvis_link", "units": "millimetres",
        "axes": {"x":"forward", "y":"left", "z":"up", "handedness":"right"},
        "pivot_m": [0,0,0], "body_to_pelvis_quaternion_xyzw": [0,0,0,1],
        "centroid_subtracted": False, "station_count": 17, "marker_count_per_player": 12,
        "marker_mode": "12mm_round_reflective_stickers", "sticker_total_thickness_mm": 0.2,
        "geometry_status": "CAD_NOMINAL_NOT_PHYSICAL_CALIBRATION", "import_validation": "PENDING_CMTRACKER_ROUNDTRIP",
        "live_geometry_verifiable_over_vrpn": False,
        "source_table": {"path": "../documents/marker_transforms_pelvis_link_stickers.csv", "sha256": sha(TABLE)},
        "runtime_catalog": {"path": "../../../hope_ws/src/hope_bringup/config/pants_profiles/profiles.json", "profile": "v4"},
        "pattern_analysis": "../documents/player_pattern_analysis.json",
        "supplied_format_reference": {"filename": "UCB_P1(1).json", "sha256": "60bfe6fd97fb8dd3656315de06b01cfcb7d255cb37d043576dfa6a0f3c408c47",
            "use": "Retain existing native fields and minMarker=6. Replace only geometry, body identity, ID and display color."},
        "unchanged_printable_sha256": {p.name: sha(p) for p in sorted((SHELL / "printable").glob("*.stl"))},
        "assets": {},
        "warnings": ["Import P1.json and P2.json, not this manifest. Internal names UCB_P1/UCB_P2 preserve HOPE routing.",
            "Seven stations are shared; arbitrary occlusion can make the patterns indistinguishable.",
            "Keep unused stations nonreflective. No centroid subtraction, Motive Y-up conversion or P2-local rotation.",
            "The matching motive_asset/P1.motive and P2.motive use the same CAD points in native Y-up metres; old all-17 imports do not match these installations.",
            "Verify native import/re-export, physical pivot, axes, world frame, units and two-body occlusion behavior before play."],
    }
    for side, color in (("P1", [38,109,180,255]), ("P2", [217,116,36,255])):
        order = report[side.lower()+"_order"]
        entry = spec["assets"][side]
        body = copy.deepcopy(json.loads((RUNTIME / entry["file"]).read_text())["UCB_"+side])
        body["markerPose"] = [float(Decimal(by_name[name][k])*1000) for name in order for k in ("x_m","y_m","z_m")]
        body["id"] = 1 if side == "P1" else 2
        body["bodyColor"] = color
        encoded = json_bytes({"UCB_"+side: body})
        (directory / (side+".json")).write_bytes(encoded)
        (RUNTIME / entry["file"]).write_bytes(encoded)
        entry.update(sha256=hashlib.sha256(encoded).hexdigest(), marker_names=order,
                     marker_positions_m=[[float(by_name[name][k]) for k in ("x_m","y_m","z_m")] for name in order])
        manifest["assets"][side] = {"file": side+".json", "body_name": "UCB_"+side, "id": body["id"],
            "marker_count": 12, "marker_order": order, "unmarked_stations": sorted(set(names)-set(order)),
            "sha256": entry["sha256"], "runtime_file": "../../../hope_ws/src/hope_bringup/config/pants_profiles/"+entry["file"]}
    (RUNTIME / "profiles.json").write_bytes(json_bytes(catalog))
    (directory / "asset_manifest.json").write_bytes(json_bytes(manifest))
    (SHELL / "documents/player_pattern_analysis.json").write_bytes(json_bytes(report))
    return manifest


def write_motive_assets(report):
    """Author native XML candidates; no learned cache or live validation invented."""
    rows, names, _, _ = geometry()
    by_name = dict(zip(names, rows))
    directory = SHELL / "motive_asset"
    template = ET.parse(SHELL / "documents/source_evidence/optitrack/A3.motive").getroot()
    manifest = {
        "format": "HOPE_MOTIVE_PLAYER_PATTERN_RELEASE", "schema_version": 1,
        "revision": "pelvis_link_nominal_player12_20261027", "date": DATE,
        "body_frame": "pelvis_link", "axes": {"x": "forward", "y": "left", "z": "up"},
        "native_units": "metres", "native_from_ros": "[x, z, -y]",
        "ros_from_native": "[x, -z, y]", "required_streaming_up_axis": "Z",
        "modeldef_y_up_to_z_up": True, "centroid_subtracted": False,
        "body_to_pelvis_xyz_m": [0, 0, 0], "body_to_pelvis_quaternion_xyzw": [0, 0, 0, 1],
        "station_count": 17, "marker_count_per_player": 12,
        "marker_mode": "12mm_round_reflective_stickers", "sticker_total_thickness_mm": 0.2,
        "geometry_status": "ALL_CAD_NOMINAL_NOT_PHYSICAL_CALIBRATION",
        "motive_import_reexport": "NOT_PERFORMED", "live_tracking": "NOT_PERFORMED",
        "source_table": {"path": "../documents/marker_transforms_pelvis_link_stickers.csv", "sha256": sha(TABLE)},
        "unchanged_printable_sha256": {p.name: sha(p) for p in sorted((SHELL / "printable").glob("*.stl"))},
        "historical_reference_sha256": {name: sha(directory / name) for name in
            ("../documents/source_evidence/optitrack/A3.motive", "../documents/source_evidence/optitrack/A3.csv")},
        "assets": {},
        "warnings": ["Import P1.motive and P2.motive, not this manifest or old all-17 files.",
            "Native XML candidates: test import and re-export in Motive before deployment; no learned constraint_graph is fabricated.",
            "Distinct names/UUIDs/streaming IDs do not resolve shared-only optical ambiguity. Test both robots together under occlusion.",
            "Resolve existing UCB_P1/UCB_P2 names and streaming ID 9/10 collisions in a backed-up project.",
            "No centroid reset, extra P2 local rotation or second sticker-thickness offset.",
            "The legacy measured stickers_v3 calibration tool still requires all 17 sites; it cannot issue a receipt for this pair."],
    }
    def native_id(label):
        value = uuid.uuid5(uuid.NAMESPACE_URL, "https://github.com/hitchopen/HOPE/mocap_sticker_shell_v3/player12/" + label).hex
        return value[:16] + ":" + value[16:]
    for side, stream_id in (("P1", 9), ("P2", 10)):
        order = report[side.lower() + "_order"]
        root = copy.deepcopy(template)
        body = root.find("./node_assets/rigid_body")
        body.set("id", native_id(side))
        body_name = "UCB_" + side
        for prop in body.find("properties"):
            key = prop.get("name")
            value = body_name if key in ("NodeName", "AssetName", "JointName") else (
                str(stream_id) if key == "UserData" else order[0] if key == "ChildInfo" else None)
            if value is not None:
                prop.set("value", value)
                if "default_value" in prop.attrib:
                    prop.set("default_value", value)
        for tag in ("constraints", "marker_sticks", "constraint_graph"):
            old = body.find(tag)
            if old is not None:
                body.remove(old)
        constraints = ET.SubElement(body, "constraints")
        markers = []
        for index, name in enumerate(order, 1):
            xyz = [float(by_name[name][k]) for k in ("x_m", "y_m", "z_m")]
            native = [xyz[0], xyz[2], -xyz[1]]
            marker_id = native_id(side + "/" + name)
            c = ET.SubElement(constraints, "constraint", name=name, id=marker_id,
                              member_id=str(index), joint_name=body_name)
            ET.SubElement(c, "offset").text = ",".join(f"{v:.9f}" for v in native)
            for key, value in (("active_id", "0"), ("diameter", "0.012"), ("weight", "1"), ("color", "#00010101")):
                ET.SubElement(c, key).text = value
            markers.append({"station": name, "member_id": index, "uuid": marker_id,
                            "position_pelvis_link_m": xyz, "position_motive_native_y_up_m": native})
        sticks = ET.SubElement(body, "marker_sticks")
        for a, b in itertools.combinations(order, 2):
            ET.SubElement(sticks, "marker_stick", origin=a, end=b, color="1 1 1")
        ET.indent(root, space="  ")
        encoded = ET.tostring(root, encoding="utf-8", xml_declaration=True) + b"\n"
        (directory / (side + ".motive")).write_bytes(encoded)
        manifest["assets"][side] = {"file": side + ".motive", "body_name": body_name,
            "uuid": body.get("id"), "streaming_id": stream_id, "sha256": hashlib.sha256(encoded).hexdigest(),
            "marker_count": 12, "marker_order": order, "unmarked_stations": sorted(set(names) - set(order)),
            "matching_chingmu_file": "../vrpn_asset/" + side + ".json", "markers": markers}
    (directory / "asset_manifest.json").write_bytes(json_bytes(manifest))
    return manifest


def pdf_instructions(report, manifest):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    from reportlab.pdfgen import canvas
    from reportlab.lib.colors import HexColor, white
    from reportlab.lib.styles import ParagraphStyle
    from reportlab.platypus import Paragraph, Table, TableStyle
    from reportlab.lib.utils import ImageReader

    rows, names, p, _ = geometry()
    p1, p2 = set(report["p1_order"]), set(report["p2_order"])
    ink, muted, blue, orange = "#182B40", "#566777", "#266DB4", "#D97424"
    w, h = 595.276, 841.89
    margin = 42
    out = SHELL / "output/pdf/A3_marker_shell_v3_instructions.pdf"
    c = canvas.Canvas(str(out), pagesize=(w,h), invariant=1)
    c.setTitle("A3 marker shell v3 - P1 and P2 installation instructions")
    c.setAuthor("HOPE")
    body_style = ParagraphStyle("body", fontName="Helvetica", fontSize=9.5, leading=14, textColor=HexColor(ink))
    small_style = ParagraphStyle("small", parent=body_style, fontSize=8, leading=11)
    page = 0
    def start(title, subtitle):
        nonlocal page
        if page: c.showPage()
        page += 1
        c.setFillColor(HexColor(ink)); c.rect(0,h-116,w,116,fill=1,stroke=0)
        c.setFont("Helvetica-Bold",9); c.setFillColor(HexColor("#9EC9EE"))
        c.drawString(margin,h-32,"HOPE  /  A3 MARKER SHELL v3")
        c.setFont("Helvetica-Bold",21); c.setFillColor(white); c.drawString(margin,h-64,title)
        c.setFont("Helvetica",10); c.drawString(margin,h-86,subtitle)
        c.setStrokeColor(HexColor("#D5DDE5")); c.line(margin,37,w-margin,37)
        c.setFont("Helvetica",8); c.setFillColor(HexColor(muted))
        c.drawString(margin,23, f"{DATE}  |  Nominal CAD geometry; import and physical verification pending")
        c.drawRightString(w-margin,23,str(page))
        return h-139
    def para(text, y, small=False):
        paragraph = Paragraph(text, small_style if small else body_style)
        _, height = paragraph.wrap(w-2*margin, 1000)
        assert y-height > 47, (page, text[:70], y-height)
        paragraph.drawOn(c,margin,y-height)
        return y-height-10
    def table(data,y,widths,font=8.5):
        tab=Table(data,colWidths=widths,repeatRows=1)
        tab.setStyle(TableStyle([("BACKGROUND",(0,0),(-1,0),HexColor(ink)),("TEXTCOLOR",(0,0),(-1,0),white),
            ("FONTNAME",(0,0),(-1,0),"Helvetica-Bold"),("FONTNAME",(0,1),(-1,-1),"Helvetica"),
            ("FONTSIZE",(0,0),(-1,-1),font),("BOTTOMPADDING",(0,0),(-1,-1),6),("TOPPADDING",(0,0),(-1,-1),6),
            ("ROWBACKGROUNDS",(0,1),(-1,-1),[white,HexColor("#EFF3F7")]),("VALIGN",(0,0),(-1,-1),"MIDDLE")]))
        _,height=tab.wrap(w-2*margin,1000); assert y-height>47,(page,y-height)
        tab.drawOn(c,margin,y-height); return y-height-14
    def figure(fig,y,height):
        stream=io.BytesIO();fig.savefig(stream,format="png",dpi=180,bbox_inches="tight",facecolor="white");plt.close(fig)
        c.drawImage(ImageReader(stream),margin,y-height,width=w-2*margin,height=height,preserveAspectRatio=True,anchor="c")
        return y-height-12
    def stl_triangles():
        meshes=[]
        dtype=np.dtype([("normal","<f4",(3,)),("vertices","<f4",(3,3)),("attribute","<u2")])
        for filename in ("a3_marker_shell_v3_half_a.stl","a3_marker_shell_v3_half_b.stl"):
            content=(SHELL/"printable"/filename).read_bytes(); count=int.from_bytes(content[80:84],"little")
            assert len(content)==84+50*count
            a=np.frombuffer(content,offset=84,dtype=dtype,count=count)["vertices"].astype(float)
            meshes.append(np.stack([a[:,:,2],a[:,:,0],a[:,:,1]-145.800095037096],axis=2))
        return np.concatenate(meshes)
    triangles=stl_triangles()
    def pattern_views(side):
        selected=set(report[side.lower()+"_order"]); color=blue if side=="P1" else orange
        fig,axes=plt.subplots(1,2,figsize=(10,4.3))
        for ax,sign,title in zip(axes,[1,-1],["FRONT - robot left is image right","REAR - robot left is image left"]):
            depth=triangles.mean(axis=1)[:,0]*sign; order=np.argsort(depth)
            projected=triangles[:,:,1:3].copy();projected[:,:,0]*=sign
            ax.add_collection(PolyCollection(projected[order],facecolors="#E5E9ED",edgecolors="none",rasterized=True))
            ids=FRONT if sign==1 else set(names)-FRONT
            for i,name in enumerate(names):
                if name not in ids:continue
                x,z=p[i,1]*sign,p[i,2]; installed=name in selected
                ax.scatter(x,z,s=110,c=color if installed else "white",edgecolors=color if installed else "#687582",linewidths=1.3,zorder=5)
                ax.annotate(name,(x,z),xytext=(0,10 if name not in {"S11","S20"} else -17),textcoords="offset points",ha="center",fontsize=9,fontweight="bold",zorder=6,
                            bbox=dict(facecolor="white",edgecolor="none",alpha=.85,pad=.8))
            ax.set(xlim=(-104,104),ylim=(-180,-27),aspect="equal",title=title,xlabel="pelvis lateral position (mm)")
            ax.set_ylabel("Z (mm)");ax.spines[["top","right"]].set_visible(False)
        fig.tight_layout();return fig

    y=start("One shell. Two player patterns.","17 mounting stations / 12 reflective stickers per robot / unchanged printable model")
    y=para("Use the existing v3 halves for both players. P1 and P2 differ only in which 12 stations receive reflective stickers. Keep the other five stations nonreflective. Both assets preserve the nominal <b>pelvis_link</b> origin and ROS axes: <b>X forward, Y left, Z up</b>.",y)
    y=table([["Install","Front / rear","CMTracker / Motive / body"],["Player 1","7 / 5","P1.json / P1.motive / UCB_P1"],["Player 2","4 / 8","P2.json / P2.motive / UCB_P2"]],y,[105,100,306])
    y=para("<b>P1, supplied selection:</b> "+", ".join(P1_ORDER)+".",y)
    y=para("<b>P2, calculated selection:</b> "+", ".join(report["p2_order"])+".",y)
    fig,ax=plt.subplots(figsize=(7,3.7))
    for i,name in enumerate(names):
        color=blue if name in p1-p2 else orange if name in p2-p1 else "#8A65A8"
        ax.scatter(p[i,1],p[i,0],color=color,s=70)
        offset={"S18":(-18,-10),"S08":(5,5),"S17":(-18,-10),"S07":(5,5),
                "S05":(5,8),"S15":(5,-10),"S16":(-12,-12),"S06":(5,-2)}.get(name,(5,4))
        ax.annotate(name,(p[i,1],p[i,0]),xytext=offset,textcoords="offset points",fontsize=8,
                    bbox=dict(facecolor="white",edgecolor="none",alpha=.8,pad=.4))
    ax.scatter(0,0,marker="+",color=ink,s=100);ax.text(3,0,"pelvis origin",fontsize=8)
    ax.set(xlim=(-105,110),ylim=(-110,105),aspect="equal",xlabel="+Y = robot left (mm)",ylabel="+X = robot forward (mm)")
    ax.set_title("Top projection: blue P1-only / orange P2-only / purple shared",fontsize=10);ax.grid(alpha=.15)
    y=figure(fig,y,240)
    para("<b>Identity limit:</b> seven locations must be shared because 12 + 12 - 17 = 7. Shared-only observations can be identical. This is an optimized candidate under the documented CAD screening criterion, not an all-view or partial-occlusion guarantee.",y)

    for side in ("P1","P2"):
        selected=report[side.lower()+"_order"]; unused=manifest["assets"][side]["unmarked_stations"]
        y=start(side+" sticker installation", "Filled circles = reflective sticker / hollow circles = nonreflective station")
        y=para("<b>Install these 12:</b> "+", ".join(selected)+".",y)
        y=para("<b>Leave these five nonreflective:</b> "+", ".join(unused)+". Do not add reflective balls, foil, tape or bright labels at these stations.",y)
        y=figure(pattern_views(side),y,245)
        y=para("The grey surfaces are projections of the unchanged printable STL files. Original Sxx labels identify physical stations; array position in the JSON is only authoring order. Drawings identify locations and do not prove camera visibility.",y,True)
        y=table([["Region","Reflective stations"],["Front",", ".join(s for s in selected if s in FRONT)],
                 ["Rear upper",", ".join(s for s in selected if s in {"S04","S05","S06","S07","S08"})],
                 ["Rear middle",", ".join(s for s in selected if s in {"S14","S15","S16","S17","S18"})]],y,[105,406])
        y=para("Centre one 12 mm round sticker on each selected flat face after trial-fitting the flush pin. Film plus adhesive totals 0.20 mm. Use the same clean surface and centring method on both robots; inspect under the mocap cameras' IR illumination.",y)
        para("Import <b>vrpn_asset/"+side+".json</b> for ChingMu or <b>motive_asset/"+side+".motive</b> for OptiTrack. Both contain the same 12 optical centres and body name <b>UCB_"+side+"</b>. Do not recenter the body or rotate its local frame to face the table.",y)

    y=start("Walk-around installation map","Unwrapped azimuth: front 0/360 deg / robot left 90 / rear 180 / right 270")
    fig,axes=plt.subplots(2,1,figsize=(10,7),sharex=True)
    angles=np.mod(np.rad2deg(np.arctan2(p[:,1],p[:,0])),360)
    for ax,side,color in zip(axes,("P1","P2"),(blue,orange)):
        selected=set(report[side.lower()+"_order"])
        for i,name in enumerate(names):
            ax.scatter(angles[i],p[i,2],s=110,facecolor=color if name in selected else "white",edgecolor=color if name in selected else "#8D98A3",linewidth=1.4)
            ax.annotate(name,(angles[i],p[i,2]),xytext=(0,9),textcoords="offset points",ha="center",fontsize=9)
        ax.set(ylim=(-177,-36),xlim=(-8,368),ylabel=side+" / Z (mm)")
        ax.set_xticks([0,90,180,270,360]);ax.grid(alpha=.18);ax.spines[["top","right"]].set_visible(False)
    axes[-1].set_xlabel("Azimuth about pelvis_link +Z (degrees)");fig.tight_layout()
    y=figure(fig,y,410)
    y=para("This is an angular index, not a print template. Use it with the front/rear views and the coordinate table; do not measure sticker placement from this drawing. Both stations S21 and S22 sit lower on the front; P2 uses S22 only.",y)
    y=para("<b>P1-only identifiers:</b> "+", ".join(report["p1_only"])+".<br/><b>P2-only identifiers:</b> "+", ".join(report["p2_only"])+".<br/><b>Shared:</b> "+", ".join(report["shared"])+".",y)
    para("All P2-only sites are on the rear. Arrange actual camera coverage so these sites can be reconstructed despite arms and torso occlusion. A front-only observation of P2 is a subset of P1's front pattern.",y)

    y=start("Optical centres and asset order","Metres in pelvis_link / same coordinates for both players / sticker thickness included")
    data=[["Station","P1 index","P2 index","X (m)","Y (m)","Z (m)"]]
    for row in rows:
        name=row["marker_id"]
        indices=[str(report[s+"_order"].index(name)+1) if name in report[s+"_order"] else "-" for s in ("p1","p2")]
        data.append([name,*indices,*[f"{float(row[k]):.6f}" for k in ("x_m","y_m","z_m")]])
    y=table(data,y,[58,59,59,111,111,113])
    y=para("Indices are 1-based XYZ triplet positions in ChingMu's <b>markerPose</b> array and member_id values in the authored Motive file. Native re-export may reorder labels. They are not VRPN sensor indices. A dash means leave the station nonreflective.",y)
    y=para("ChingMu stores ROS-axis coordinates in millimetres. Motive stores [X, Z, -Y] in native Y-up metres. Both preserve full CSV precision. The 17-row CSVs and workbook remain unchanged. Display rounding is not a physical accuracy claim.",y)
    para("The intended fixed body-to-pelvis transform is translation [0,0,0] m and quaternion [0,0,0,1] (xyzw). The streamed world-to-body pose is dynamic. Do not subtract either pattern's centroid or apply an extra P2 rotation.",y)

    y=start("Station hardware and printing","Existing flat 12 mm seat / blind screw bore / side-loaded captive nut")
    y=table([["Feature","Existing nominal dimension"],["Sticker support face","12.0 mm diameter; flat, no raised rim"],["Support foot","20.0 mm tapered foot below optical plane"],["Central bore","3.4 mm diameter; 4.6 mm blind axial depth"],["Flush pin","3.45 mm diameter x 1.30 mm length"],["Sticker + adhesive","12 mm round x 0.20 mm total thickness"],["Nut pocket","5.8 mm across flats x 2.65 mm high"],["Pocket top","1.5 mm below seat; outside side access"],["Provisional nut","M3; 5.5 mm across flats, about 2.4 mm thick"]],y,[190,321])
    y=para("<b>Sticker mode:</b> use 12 reflective stickers per robot. Trial-fit each required pin flush; nominal interference is provisional. Do not force a pin or sand the optical support plane. Unselected stations may be left bare or capped with nonreflective material; verify they do not appear as markers under IR.",y)
    y=para("<b>Ball mode:</b> the CAD retains its existing mounting feature, but these P1/P2 assets describe stickers, not sphere centres. Remove sticker/pin, verify the actual ball thread, nut retention and stud reach. Candidate stud penetration must not exceed 4.0 mm below the seat without a new hardware check. Do not drill through the pelvis-facing blind floor.",y)
    y=para("A 12 mm ball diameter does not establish its thread or mounted centre. Measure each ball/stud/spacer offset before authoring ball assets; do not reuse these sticker coordinates or the old 6 mm registration stand-off. Neither mode has a qualified crash-survival rating.",y)
    para("Print the two existing halves separately at 100% scale in millimetres. Approximate footprints are 176 x 194 mm before brim/support margins. In OpenSCAD use F5 for assembly preview; select half_a or half_b for F6/export. With bed_oriented=true each opens toward the bed at Z=0. Print placement does not change the marker coordinates.",y)

    y=start("Fit and installation checks","Same manufacturing masters / different optical populations")
    for text in [
        "<b>1. Inspect slicing.</b> Check thin ribs, bridges, seams, structural screws, blind bores and side nut slots. Keep support scars off the flat marker faces. No printer/material-specific qualification is supplied.",
        "<b>2. Preserve the interior.</b> The shell geometry and fastener holes are unchanged. Fit on a secured robot and inspect pelvis clearance, cables, ventilation, seam closure and full joint travel. Do not create new through-holes.",
        "<b>3. Populate the correct pattern.</b> Use pages 2-4 and check every Sxx station twice. Each robot has exactly 12 reflective stickers. Distinguish the shells with a nonreflective P1/P2 label outside the optical seats.",
        "<b>4. Validate each imported body.</b> Confirm the 12 selected positions, pelvis origin and axes after a native export. Test the two bodies separately first, then together. Any all-17 ChingMu/Motive definition is incompatible with these 12-sticker installations.",
        "<b>5. Challenge occlusion.</b> Check front, rear and side camera coverage; arms, torso, low grazing views and reacquisition after complete loss. Intentionally expose only shared stations to demonstrate the ambiguity limit. Record wrong-name matches, pose jumps and dropouts.",
        "<b>6. Check the physical pose.</b> Compare the streamed pelvis pivot and local axes against physical datums and a measured displacement. Record residual error; nominal CAD coordinates are not measured calibration. Verify server world/table axes and units separately.",
        "<b>7. Keep the limitation visible.</b> Six-marker tracking does not prove player identity; absence of another marker is not reliable evidence of its pattern under occlusion. Accept a result only after the actual camera setup has been checked.",
    ]: y=para(text,y)
    y=para("<b>Existing mesh evidence (2026-10-11, not rerun for this optical-only change):</b> front 74,536 triangles, rear 99,132; each is a closed, consistently wound component with no detected self-intersections. Rear-seat probes checked flat tops, diameters and blind depths. The STL hashes are unchanged and recorded in asset_manifest.json.",y,True)
    para("Historical local wall sampling found roughly 2.5 mm at most restored sites, but about 1.50 mm at an old front f1 edge. Preserve that region in slicing. Those checks are sampled evidence, not a certified global thickness bound or physical fit test. See geometry_repair_status.md and printability_validation.json.",y,True)

    y=start("Import P1 and P2 into ChingMu","Two files / distinct body names and IDs / identical pelvis frame convention")
    for text in [
        "<b>1. Back up the CMTracker/MCServer project.</b> Import vrpn_asset/P1.json and vrpn_asset/P2.json into a test project. These follow the supplied CMTracker JSON schema; they are not generic VRPN import files. Do not import asset_manifest.json or player_pattern_analysis.json.",
        "<b>2. Confirm identities.</b> P1.json contains UCB_P1, ID 1; P2.json contains UCB_P2, ID 2. File names are P1/P2, while the internal UCB_ prefix preserves HOPE topic routing. Resolve ID collisions before use. Asset IDs do not declare VRPN sensor indices.",
        "<b>3. Preserve the frame.</b> markerPose uses millimetres in pelvis_link; Rbc remains identity. Keep X forward, Y left, Z up. Do not recenter, mirror, reset orientation to the table, apply Motive Y-up mapping or add sticker thickness again.",
        "<b>4. Export and compare.</b> Confirm exactly 12 XYZ triplets and the correct station list for each body. A reorder can be equivalent, but the geometry, pivot and axes must remain equivalent. These files have offline checks, not a native import certificate.",
        "<b>5. Match the HOPE profile.</b> Install the companion updated V4 profiles and select ChingMu/VRPN, V4, and the correct table side while stopped. V4 is the software alias for this v3 shell. The profile now carries a separate 12-marker definition for each side. Do not pair these imports with the old identical 17-point profiles.",
        "<b>6. Verify the stream.</b> VRPN sends poses, not a ModelDef or a remotely measured asset hash. Check measured displacement to select scale 0.001 for millimetres or 1.0 for metres. Confirm the table world frame independently: Z up, tabletop Z=0, X in [0,2.74], Y in [-1.525,0] m.",
        "<b>7. Preserve P2 local axes.</b> The same local coordinate convention applies to both robots. The existing runtime table-side transform handles P2's world coordinates; do not bake that 180-degree rotation into P2.json.",
    ]: y=para(text,y)
    para("The inherited minMarker=6 is retained; it is not an identity guarantee. OptiTrack uses the matching P1/P2 Motive pair described next. See vrpn_asset/README.md for ChingMu commissioning and reproduction commands.",y)

    y=start("Import P1 and P2 into OptiTrack", "Matching sticker subsets / native Y-up storage / ROS Z-up stream")
    for text in [
        "<b>1. Back up the Motive project.</b> Test-import motive_asset/P1.motive and P2.motive in an isolated project. These native XML candidates have offline checks only; Motive import/re-export and physical tracking are not yet verified. Do not import the old all-17 assets for these installations.",
        "<b>2. Confirm two distinct bodies.</b> P1.motive is UCB_P1, streaming ID 9; P2.motive is UCB_P2, ID 10. Each has 12 constraints, member IDs 1-12, station names Sxx and new distinct UUIDs. Resolve existing body-name and streaming-ID collisions before use.",
        "<b>3. Preserve the pelvis frame.</b> All points are nominal CAD sticker centres, not a new camera measurement. Native offsets are [ROS X, ROS Z, -ROS Y] in metres. Set Streaming Up Axis to Z Up and keep modeldef_y_up_to_z_up=true in the adapter. Do not apply that conversion twice.",
        "<b>4. Keep identity body-to-pelvis extrinsics.</b> Do not reset pivot location/orientation, subtract the marker centroid, add sticker thickness again, mirror P2 or bake a table-side rotation into its local coordinates. Both body-local origins represent pelvis_link with X forward, Y left, Z up after conversion.",
        "<b>5. Export and verify.</b> Re-export each body; compare all 12 offsets (allow reordered labels), pivot, axes and IDs with asset_manifest.json. No learned constraint_graph was invented. If Motive rejects the candidate, use the manifest's native coordinates to author the body through supported Motive tooling, then repeat the export check.",
        "<b>6. Verify the ROS path.</b> Default motive_live accepts the selected live ModelDef without a file pin. Optional motive_asset mode now checks the selected player's exact 12-point reference. This is a nominal geometry check, not proof of physical fit or identity under occlusion. The legacy measured stickers_v3 capture still requires all 17 sites and is not suitable for this pair.",
        "<b>7. Qualify the installation.</b> Check measured pelvis displacement, axis directions, table/world registration and simultaneous two-body tracking. Test shared-only views, loss and reacquisition. Names and UUIDs cannot resolve identical observed subsets; these patterns have seven common stations.",
    ]: y=para(text,y)
    para("See motive_asset/README.md and asset_manifest.json for hashes, coordinates and validation limits. Vendor references: docs.optitrack.com/motive/rigid-body-tracking and docs.optitrack.com/motive/data-streaming. Original measured exports remain unchanged as historical evidence, not current imports.",y,True)

    y=start("How P2 was selected","Exhaustive CAD screening / explicit design assumptions / partial-view limits")
    y=para("There are 6,188 ways to choose 12 of 17 stations. The minimum possible overlap with fixed P1 is seven, so P2 must use all five P1-absent rear sites. This leaves 792 maximum-separation candidates, all evaluated by the reproducible authoring tool.",y)
    y=para("Rank in order: (1) minimize nonshared four-point distance-signature candidates within 2 mm; (2) minimize nonshared three-point candidates at 2 mm; (3) maximize worst P2 normal-facing count; (4) maximize views containing a P1-only point; (5) maximize views with six P2-facing points, then total facing count and P1-only redundancy. A station-order tie break makes the choice deterministic.",y)
    y=table([["CAD metric","Selected P2 result"],["Overlap / exclusive per player","7 shared / 5 unique each"],["Nonshared 4-point candidates, 2 mm","0 (conservative distance-signature screen)"],["Nonshared 3-point candidates, 2 mm","6"],["Same screen at 3 / 5 mm, four points","1 / 24 candidates"],["Same screen at 3 / 5 mm, three points","15 / 70 candidates"],["P2 normal-facing count, 288 views","minimum 2; mean 3.934"],["Views with P1-only / P2-only facing point","286 / 178 of 288"]],y,[265,246],8)
    y=para("Signature screening compares sorted inter-point distances; it is a necessary rather than sufficient condition for a rigid fit. The 2 mm edge-distance threshold is a design assumption, not measured CMTracker accuracy. Larger tolerances admit more similarities. No camera layout, torso/arm occlusion, shell ray tracing or triangulation is included.",y)
    y=para("Facing grid: azimuth 0-355 degrees in 5-degree steps, elevation 0/15/30/45 degrees; outward-normal incidence at most 60 degrees. These are upper bounds on one-direction marker availability, not reconstructed marker counts. Cameras must supply multiple views. The analysis JSON also reports 45/70-degree sensitivity.",y)
    para("<b>Unavoidable ambiguity:</b> the shared seven contain 35 identical triples, 35 identical quadruples and seven identical six-point subsets. Eight genuinely observed matched stations would exclude a shared-only observation in the nominal geometry; this is not a vendor recognition guarantee, and no eight-point gate is imposed. No pattern satisfying this request can be certified unambiguous from every partial view.",y)

    y=start("CAD datum and mount-seat reference","Geometry unchanged / optical-only revision / original station IDs retained")
    y=para("The existing nominal registration is preserved: pelvis X = CAD Z / 1000, pelvis Y = CAD X / 1000, pelvis Z = (CAD Y - 145.800095037096) / 1000. The CAD source uses millimetres and CAD Y up. The marker datum is pelvis_link, not either subset's centroid.",y)
    with (SHELL/"documents/marker_transforms_pelvis_link_mount_seats.csv").open(newline="") as stream:seats=list(csv.DictReader(stream))
    data=[["Station","Seat X (m)","Seat Y (m)","Seat Z (m)"]]
    for row in seats:data.append([row["marker_id"],*[f"{float(row[k]):.6f}" for k in ("x_m","y_m","z_m")]])
    y=table(data,y,[70,147,147,147],8)
    y=para("Sticker centre = seat + 0.20 mm along the outward normal. The CSVs and workbook contain complete translation, quaternion (xyzw) and normal data. Each marker-local frame maps into pelvis_link; local +Z is outward. Round-sticker roll is an authoring convention, not an optical measurement.",y,True)
    para("The registration derives from nominal CAD references, with numerical consistency checks documented in CAD_registration.md. The old measured Motive subset has 1.395 mm RMS / 1.853 mm maximum fit residual; five restored rear sites were not observed in that export. Those records neither validate the new P2 installation nor establish physical pelvis accuracy. Keep the original source exports and geometry evidence for traceability.",y,True)
    c.save()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pdf",action="store_true",help="also rebuild the instruction PDF")
    parser.add_argument("--analyze-only",action="store_true",help="print the computed selection without writing files")
    args=parser.parse_args()
    report=choose_pattern()
    if args.analyze_only:
        print(json.dumps(report,indent=2));return
    manifest=write_assets(report)
    write_motive_assets(report)
    if args.pdf:pdf_instructions(report,manifest)
    print("P1:",", ".join(report["p1_order"]))
    print("P2:",", ".join(report["p2_order"]))
    print("Shared:",", ".join(report["shared"]))


if __name__=="__main__":
    main()
