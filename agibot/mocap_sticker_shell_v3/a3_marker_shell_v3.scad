// A3 v3: geometry-checked printable meshes, millimetres.
// Physical fit, material/process, hardware retention and crash testing remain required.
// Assembly display is rotated for CAD-Y-up viewing; individual STL masters retain
// their original CAD frame, which is used by the pelvis_link marker tables.

part = "assembly"; // [assembly,half_a,half_b,flush_pin]
bed_oriented = true;

module half_a() { import("printable/a3_marker_shell_v3_half_a.stl", convexity=20); }
module half_b() { import("printable/a3_marker_shell_v3_half_b.stl", convexity=20); }

if (part == "assembly") {
    assert($preview, "Export half_a and half_b separately, not the assembled pair.");
    rotate([90,0,0]) {
        color([0.31,0.56,0.69]) half_a();
        color([0.49,0.61,0.55]) half_b();
    }
} else if (part == "half_a") {
    translate([0,0,bed_oriented ? 9.796319961548 : 0]) half_a();
} else if (part == "half_b") {
    if (bed_oriented) translate([0,0,3.500000715256]) rotate([0,180,0]) half_b();
    else half_b();
} else if (part == "flush_pin") {
    import("printable/a3_marker_shell_v3_flush_pin.stl", convexity=10);
} else assert(false,"Choose assembly, half_a, half_b, or flush_pin.");
