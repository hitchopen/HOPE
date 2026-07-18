# PKU (Peking University) Open-Source Hardware

Two robot mounting parts open-sourced by the Peking University team, shared as reference for all HOPE challenge participants:

1. **Hip reflective-marker mount** (3D-printed shell) for motion-capture localization of the robot
2. **Wrist racket adapter** — a metal connector that reproduces the human grip angle on a table-tennis racket

---

## 1. Hip reflective-marker 3D-printed shell

Directory: [hip_marker_shell/](hip_marker_shell/)

- Latest-version 3D-printed shell for the robot hip, holding the reflective marker balls used by the mocap system to localize the hip.
- **The marker balls must be wrapped with reflective film.**
- Printed version: **Li Xianglong / Zhou Fuxing, latest revision** (see [hip_marker_shell_printed_latest.png](hip_marker_shell/hip_marker_shell_printed_latest.png)).

### Marker coordinates (v2)

Layout and coordinates: [hip_marker_layout_and_coords_v2.png](hip_marker_shell/hip_marker_layout_and_coords_v2.png). Five front markers (f1–f5) and five back markers (b1–b5):

| joint         | origin_x | origin_y | origin_z |
|---------------|---------:|---------:|---------:|
| ball_f1_joint |   0.09   |   0      |  -0.13   |
| ball_f2_joint |   0.08   |   0.05   |  -0.14   |
| ball_f3_joint |   0.08   |  -0.05   |  -0.14   |
| ball_f4_joint |   0.078  |  -0.03   |  -0.18   |
| ball_f5_joint |   0.078  |   0.03   |  -0.18   |
| ball_b1_joint |  -0.09   |   0      |  -0.1    |
| ball_b2_joint |  -0.085  |   0.055  |  -0.13   |
| ball_b3_joint |  -0.085  |  -0.055  |  -0.13   |
| ball_b4_joint |  -0.085  |  -0.03   |  -0.18   |
| ball_b5_joint |  -0.085  |   0.03   |  -0.18   |

> **Note (v2 change):** the outer contour of the hip shell was revised. At its original position the front f1 marker
> interferes with the shell, so its X position was moved from **0.08 to 0.09** (the table above already reflects the
> adjusted value).

## 2. Wrist racket adapter

Directory: [wrist_racket_adapter/](wrist_racket_adapter/)

- Metal wrist connector that joins the gripping hand and the table-tennis racket, reproducing the human grip angle
  (see [wrist_racket_adapter_photo.png](wrist_racket_adapter/wrist_racket_adapter_photo.png)).
- **Must stay consistent with the URDF provided by AgiBot — in particular, verify the racket pose relative to the wrist.**
- This part is shared as the standard solution for the nine other participating universities; **fabrication is handled by AgiBot.**
