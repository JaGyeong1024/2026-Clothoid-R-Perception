#!/usr/bin/env python3
"""2026_pipeline.drawio 생성기. 격자 좌표로 배치하고 모든 엣지를 직각으로 그린다.
사용: python3 tools/gen_pipeline_drawio.py > 2026_pipeline.drawio
"""
import html, itertools

_id = itertools.count(10)
cells = []

def cell(value, style, x, y, w, h, parent="1", cid=None):
    cid = cid or f"c{next(_id)}"
    cells.append(f'<mxCell id="{cid}" value="{html.escape(value, quote=True)}" style="{style}" vertex="1" parent="{parent}">'
                 f'<mxGeometry x="{x}" y="{y}" width="{w}" height="{h}" as="geometry"/></mxCell>')
    return cid

def tlabel(text, cx, cy, w=130, h=24):
    """절대좌표 라벨 (중심 cx, cy), 배경 투명"""
    return cell(text, "text;html=1;align=center;verticalAlign=middle;fontSize=9;fontStyle=1;strokeColor=none;fillColor=none;spacing=0;", cx - w // 2, cy - h // 2, w, h)

def edge(src, tgt, exit_, entry, points=(), label="", color="#000000", dashed=False, label_pos=(0, 0), cid=None):
    cid = cid or f"e{next(_id)}"
    ex, ey = exit_; nx_, ny_ = entry
    style = ("edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;endArrow=classic;endFill=1;"
             f"strokeColor={color};strokeWidth=1.2;exitX={ex};exitY={ey};entryX={nx_};entryY={ny_};"
             "exitDx=0;exitDy=0;entryDx=0;entryDy=0;" + ("dashed=1;" if dashed else ""))
    pts = "".join(f'<mxPoint x="{px}" y="{py}"/>' for px, py in points)
    arr = f"<Array as=\"points\">{pts}</Array>" if pts else ""
    cells.append(f'<mxCell id="{cid}" style="{style}" edge="1" parent="1" source="{src}" target="{tgt}">'
                 f'<mxGeometry relative="1" as="geometry">{arr}</mxGeometry></mxCell>')
    if label and label_pos is not None:
        lx, ly = label_pos
        cells.append(f'<mxCell id="{cid}l" value="{html.escape(label, quote=True)}" '
                     'style="edgeLabel;html=1;align=center;verticalAlign=middle;resizable=0;points=[];fontSize=9;fontStyle=1;labelBackgroundColor=#ffffff;" '
                     f'vertex="1" connectable="0" parent="{cid}"><mxGeometry x="{lx}" y="{ly}" relative="1" as="geometry"/></mxCell>')
    return cid

F = 'font-size: 10px;'
def desc(s):  return f'<font style="{F}">Description:<br>{s}</font>'
def func(ls): return f'<font style="{F}">function:<br>' + "<br>".join("- " + l for l in ls) + "</font>"
def topic(t, ty): return f"{t}<br>[{ty}]"

def node(title, description, functions, x, y, w=200, desc_h=40, func_h=None):
    """swimlane 노드 박스. 반환: (id, height)"""
    func_h = 0 if not functions else (func_h or (15 * (len(functions) + 1) + 16))
    h = 30 + desc_h + func_h
    nid = cell(f'<b style="font-size: 11px;">{title}</b>',
               "swimlane;fontStyle=0;childLayout=stackLayout;horizontal=1;startSize=30;horizontalStack=0;resizeParent=1;"
               "resizeParentMax=0;resizeLast=0;collapsible=0;marginBottom=0;html=1;fillColor=#ffffff;strokeColor=#333333;", x, y, w, h)
    cell(desc(description), "text;align=left;verticalAlign=middle;spacingLeft=4;spacingRight=4;overflow=hidden;html=1;strokeColor=none;fillColor=none;",
         0, 30, w, desc_h, parent=nid)
    if functions:
        cell(func(functions), "text;align=left;verticalAlign=top;spacingLeft=4;spacingRight=4;spacingTop=2;overflow=hidden;html=1;strokeColor=none;fillColor=none;",
             0, 30 + desc_h, w, func_h, parent=nid)
    return nid, h

def group(label, x, y, w, h, fill):
    return cell(label, f"rounded=0;whiteSpace=wrap;html=1;fillColor={fill};strokeColor=none;verticalAlign=top;align=left;spacingLeft=8;spacingTop=4;fontStyle=1;fontSize=11;", x, y, w, h)

def note(text, x, y, w, h=16):
    return cell(text, "text;html=1;align=left;verticalAlign=middle;fontSize=9;fontColor=#555555;strokeColor=none;fillColor=none;spacingLeft=8;", x, y, w, h)

# ---------------- 배치 상수 ----------------
R1, R2, R3 = 190, 450, 740                      # 행 중심 y: velodyne / livox / camera
SX, SW = 40, 140                                # 센서(하드웨어) 열
DX, DW = 250, 180                               # system_ws 드라이버 노드 열
PX, PW = 570, 200                               # perception 노드 열
FX = 900                                        # fusion 노드 x
BUS = 1170                                      # planning 으로 모이는 버스 x
LPX = 1220                                      # local_path 노드 x

# 프레임 / 제목
cell("", "rounded=0;whiteSpace=wrap;html=1;fillColor=#ffffff;strokeColor=#999999;", 0, 0, 1440, 940)
cell("2026 Clothoid-R Perception pipeline  ·  updated 2026-09-17",
     "text;html=1;align=left;verticalAlign=middle;fontSize=11;fontColor=#666666;strokeColor=none;fillColor=none;", 20, 8, 500, 20)

# 워크스페이스 컨테이너
cell("system_ws", "rounded=0;whiteSpace=wrap;html=1;fillColor=#f7f7f7;strokeColor=#888888;dashed=1;verticalAlign=top;align=left;spacingLeft=8;spacingTop=4;fontStyle=1;fontSize=12;", 220, 40, 240, 830)
cell("perception_ws", "rounded=0;whiteSpace=wrap;html=1;fillColor=#f5f5f5;strokeColor=#666666;verticalAlign=top;align=left;spacingLeft=8;spacingTop=4;fontStyle=1;fontSize=12;", 500, 40, 640, 830)
cell("planning_ws", "rounded=0;whiteSpace=wrap;html=1;fillColor=#f5f5f5;strokeColor=#666666;verticalAlign=top;align=left;spacingLeft=8;spacingTop=4;fontStyle=1;fontSize=12;", 1200, 350, 220, 180)

# 센서 (하드웨어)
sens_style = "rounded=0;whiteSpace=wrap;html=1;strokeColor=none;fontStyle=1;fontSize=11;fillColor="
s_vel = cell("Velodyne<br>VLP-16 LiDAR", sens_style + "#dae8fc;", SX, R1 - 40, SW, 80)
s_liv = cell("Livox<br>Horizon LiDAR", sens_style + "#dae8fc;", SX, R2 - 40, SW, 80)
s_cam = cell("SF3324-100<br>Camera", sens_style + "#f8cecc;", SX, R3 - 40, SW, 80)

# system_ws 드라이버 노드 (행 중심에 정렬)
def set_y(nid, y):
    i = next(k for k, c in enumerate(cells) if f'id="{nid}"' in c)
    cells[i] = cells[i].replace('y="0"', f'y="{y}"', 1)
d_vel, h = node("velodyne_pointcloud", "VLP-16 driver", ["UDP packets → PointCloud2", "publish /velodyne_points"], DX, 0, DW); set_y(d_vel, R1 - h // 2)
d_liv, h = node("livox_ros_driver", "Livox Horizon driver", ["SDK stream → PointCloud2", "publish /livox/lidar"], DX, 0, DW); set_y(d_liv, R2 - h // 2)
d_cam, h = node("camera_start_node", "Webcam capture + undistort", ["V4L2 / GStreamer open", "undistort (map cached)", "auto-reopen on empty frames", "publish raw + compressed"], DX, 0, DW); set_y(d_cam, R3 - h // 2)

# perception_ws 노드 + 패키지 그룹
def pkg(label, fill, nid_h, extra=None, x=PX):
    nid, h = nid_h
    # 노드 y 는 이미 확정. 그룹은 노드 좌표에서 역산
    line = [c for c in cells if f'id="{nid}"' in c][0]
    y = int(line.split('y="')[1].split('"')[0])
    gh = 40 + h + 20 + (20 if extra else 0)
    g = group(label, x - 20, y - 40, 240, gh, fill)
    cells.insert(cells.index([c for c in cells if f'id="{nid}"' in c][0]), cells.pop())  # 그룹을 노드 뒤(아래층)로
    if extra:
        note(extra, x - 20, y + h + 4, 240)
    return nid

vb_h = 30 + 40 + 15 * 5 + 16
n_vb = node("velodyne_bev_detection", "BEV-based LiDAR DL detection",
            ["Voxel BEV raster", "YOLO (velodyne_v6.pt)", "OC-SORT tracker", "stale-input heartbeat"], PX, R1 - vb_h // 2)[0]
pkg("velodyne_detection", "#d5e8d4", (n_vb, vb_h), "also: /perception/velodyne/markers [MarkerArray], bev_image [Image]")

cl_h = 30 + 40 + 15 * 6 + 16
n_cl = node("livox_euclidean_clustering", "Euclidean clustering on Livox cloud",
            ["pitch calib / ROI", "voxel + DROR", "ground removal (grid + RANSAC)", "euclidean clustering + size gate", "KF tracking (min_hits gate)"], PX, R2 - cl_h // 2)[0]
pkg("livox_clustering", "#e1d5e7", (n_cl, cl_h), "also: /perception/livox/preprocessed [PointCloud2]")

yo_h = 30 + 40 + 15 * 5 + 16
n_yo = node("yolo_detect_node", "YOLO26 pruned camera detection",
            ["YOLO26 inference (best.pt)", "class confidence gate", "NMS + containment postprocess", "2D bbox publish"], PX, R3 - yo_h // 2)[0]
pkg("yolo26", "#dae8fc", (n_yo, yo_h))

# fusion: 입력 3개가 왼쪽 변 1/4·1/2·3/4 에 들어오도록 y 결정. livox 선은 clustering/yolo 그룹 사이 틈(y=585)으로 지나감
fu_h = 30 + 40 + 15 * 6 + 16
LIV_FUS_Y = 600
fu_y = LIV_FUS_Y - fu_h // 4
n_fu = node("livox_camera_fusion_node", "Livox-camera YOLO fusion",
            ["ApproxTime sync", "LiDAR → camera projection", "ground removal (grid + RANSAC)", "YOLO bbox ROI clustering", "3D size gate + KF tracking"], FX, fu_y)[0]
pkg("livox_camera_fusion", "#fff2cc", (n_fu, fu_h), "also: /perception/fusion/filtered_cloud [PointCloud2]", x=FX)
fu_in = [fu_y + fu_h * k for k in (0.25, 0.5, 0.75)]

# planning: local_path 노드. 왼쪽 변 1/2 가 clustering 행 중심(R2)에 오도록
lp_h = 120
lp_y = R2 - lp_h // 2
n_lp = node("local_path_node", "local path generation<br>(planning_ws, 타 담당)", [], LPX, lp_y, 180, desc_h=90)[0]
lp_in = [lp_y + lp_h * k for k in (0.25, 0.5, 0.75)]

# ---------------- 엣지 ----------------
BLUE, RED, GRAY = "#1a56c4", "#b31d5c", "#333333"
GAP_SD = (SX + SW + DX) // 2          # 센서-드라이버 사이 중앙 x
GAP_DP = (DX + DW + PX) // 2          # 드라이버-perception 사이 중앙 x
GAP_PB = (PX + PW + BUS) // 2         # perception-버스 사이 중앙 x
CAM_BOT = 880                         # 카메라 이미지 우회선 y
# 센서 → 드라이버
for s_, d_, r, t in ((s_vel, d_vel, R1, "UDP"), (s_liv, d_liv, R2, "Ethernet"), (s_cam, d_cam, R3, "GMSL / USB")):
    edge(s_, d_, (1, .5), (0, .5), label_pos=None); tlabel(t, GAP_SD, r - 12, w=70, h=16)
# 드라이버 → perception
edge(d_vel, n_vb, (1, .5), (0, .5), color=BLUE, label_pos=None); tlabel(topic("/velodyne_points", "PointCloud2"), GAP_DP, R1 - 16, w=110)
edge(d_liv, n_cl, (1, .5), (0, .5), color=BLUE, label_pos=None); tlabel(topic("/livox/lidar", "PointCloud2"), GAP_DP, R2 - 16, w=110)
edge(d_cam, n_yo, (1, .5), (0, .5), color=RED, label_pos=None); tlabel(topic("/camera/image_raw/compressed", "CompressedImage"), GAP_DP, R3 - 16, w=126)
# livox → fusion (x=480 분기, 그룹 사이 틈으로)
edge(d_liv, n_fu, (1, .5), (0, .25), points=[(480, R2), (480, LIV_FUS_Y)], color=BLUE, label_pos=None)
cell("", "ellipse;fillColor=#1a56c4;strokeColor=none;", 477, R2 - 3, 6, 6)
tlabel(topic("/livox/lidar", "PointCloud2"), (480 + FX) // 2, LIV_FUS_Y, w=110)
# yolo → fusion
JOG = (PX + PW + FX) // 2
edge(n_yo, n_fu, (1, .5), (0, .5), points=[(JOG, R3), (JOG, fu_in[1])], color=RED, label_pos=None)
tlabel(topic("/perception/camera/yolo", "detect_msgs/Yolo_Objects"), JOG, R3 + 18, w=130)
# 카메라 이미지 → fusion (아래로 우회)
edge(d_cam, n_fu, (.5, 1), (0, .75), points=[(DX + DW // 2, CAM_BOT), (JOG + 15, CAM_BOT), (JOG + 15, fu_in[2])], color=RED, label_pos=None)
tlabel(topic("/camera/image_raw/compressed", "CompressedImage"), (DX + DW // 2 + JOG) // 2, CAM_BOT - 16, w=130)
# perception → planning (버스)
edge(n_vb, n_lp, (1, .5), (0, .25), points=[(BUS, R1), (BUS, lp_in[0])], label_pos=None)
tlabel(topic("/perception/velodyne/centroids", "sensor_msgs/PointCloud"), GAP_PB, R1 - 16, w=140)
edge(n_cl, n_lp, (1, .5), (0, .5), label_pos=None)
tlabel(topic("/perception/livox/centroids", "sensor_msgs/PointCloud"), GAP_PB, R2 - 16, w=140)
edge(n_fu, n_lp, (1, .5), (0, .75), points=[(BUS, fu_in[1]), (BUS, lp_in[2])], label_pos=None)
tlabel(topic("/perception/fusion/centroids", "sensor_msgs/PointCloud"), BUS + 80, (fu_in[1] + lp_in[2]) // 2 + 20, w=140)

print('<mxfile host="app.diagrams.net"><diagram name="pipeline" id="pipeline-2026">'
      '<mxGraphModel dx="1400" dy="900" grid="1" gridSize="10" guides="1" tooltips="1" connect="1" arrows="1" fold="1" page="1" pageScale="1" pageWidth="1440" pageHeight="940" math="0" shadow="0">'
      '<root><mxCell id="0"/><mxCell id="1" parent="0"/>' + "".join(cells) + '</root></mxGraphModel></diagram></mxfile>')
