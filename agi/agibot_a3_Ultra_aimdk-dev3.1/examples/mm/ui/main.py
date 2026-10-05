from PySide6.QtWidgets import QApplication, QDialog, QGraphicsScene
from PySide6.QtCore import Qt, Signal, QEvent, QPointF
from PySide6.QtGui import QPixmap, QPen, QBrush, QColor, QPolygonF
from map_ui import Ui_Dialog
import json
import subprocess
import threading
import time
import base64
import math
import socket
import yaml

class MapDialog(QDialog, Ui_Dialog):

    # 拖动画布相关变量
    _panning = False
    _pan_start = None
    map_data_ready = Signal(bytes)

    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.ssh_process = None 
        self.map_fetch_thread = None # 建图数据获取线程
        self.map_fetch_stop = threading.Event() # 建图数据获取线程停止信号
        self.map_scene = QGraphicsScene(self.graphicsView) # 创建地图场景
        self.graphicsView.setScene(self.map_scene) # 设置graphicsView使用的场景
        self.map_pixmap_item = None # 当前地图的pixmap item

        #底图点信息设置
        self.drawing_enabled = False # 是否处于绘制模式
        self.drawn_points: list[QPointF] = [] # 已绘制的点坐标列表        
        self.point_items = [] # 已绘制的点的图形项列表
        self.polygon_item = None  # 已绘制的多边形图形项
        self.point_pen = QPen(QColor("#ff5f5f"), 2) #点的边框颜色和宽度
        self.point_brush = QBrush(QColor(255, 95, 95, 180)) #点的填充颜色
        self.polygon_pen = QPen(QColor("#ff5f5f"), 2) #多边形边框颜色和宽度
        self.polygon_brush = QBrush(QColor(255, 95, 95, 80)) #多边形填充颜色
        self.map_data_ready.connect(self._update_map_scene) # 连接地图数据更新信号
        self.graphicsView.viewport().installEventFilter(self) # 安装事件过滤器以处理鼠标事件

        # 连接按钮点击信号
        self.create_map_button.clicked.connect(self.on_create_map)
        self.canncel_map_button.clicked.connect(self.on_cancel_map)
        self.save_map_button.clicked.connect(self.on_save_map_dialog)
        self.connect_button.clicked.connect(self.on_connect)
        self.draw_button.clicked.connect(self.toggle_draw_mode)
        self.select_map_button.clicked.connect(self.on_select_map)
        self.delete_map_button.clicked.connect(self.on_delete_map_dialog)
        self.relocation_button.clicked.connect(self.on_relocalization)
        self.unknow_button.clicked.connect(self.on_unknow_area)
        self.get_map_list_button.clicked.connect(self.get_map_list)
        self.clear_button.clicked.connect(self.on_clear_draw)
        self.rename_map_button.clicked.connect(self.on_rename_map)

        self.ip_input.setText("192.168.35.98") # 默认IP地址，方便测试

        self.location_thread = None # 实时位置更新线程
        self.location_stop = threading.Event() # 实时位置更新线程停止信号
        # 初始化socket，accept放到线程中
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        HOST = '0.0.0.0'
        PORT = 5001
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((HOST, PORT))
        self.sock.listen()
        self.conn = None
        self.addr = None
        threading.Thread(target=self._accept_conn, daemon=True).start()

    def _accept_conn(self):
        print("Waiting for connection...")
        conn, addr = self.sock.accept()
        print("Connected:", addr)
        self.conn = conn
        self.addr = addr

    def on_save_map_dialog(self):
        from PySide6.QtWidgets import QInputDialog
        map_name, ok = QInputDialog.getText(self, "输入地图名", "请输入新地图名称：")
        if ok and map_name.strip():
            self.on_save_map(map_name.strip())
        else:
            self.show_some.setText("未输入地图名，未保存地图")

    def on_relocalization(self):
        current_item = self.map_list.currentItem()
        if current_item is None:
            self.show_some.setText("请先选择一个地图进行重定位")
            return
        text = current_item.text()
        if '(' in text and ')' in text:
            map_id = text.split('(')[-1].split(')')[0].strip()
        else:
            map_id = text.strip()
        self.relocalization(map_id)

    def on_select_map(self):
        # 获取选中项
        item = self.map_list.currentItem()
        if item is None:
            self.show_some.setText("请先选择一个地图")
            return
        text = item.text()
        # 假设格式为“地图名 (map_id)”
        if '(' in text and ')' in text:
            map_id = text.split('(')[-1].split(')')[0].strip()
            map_name = text.split('(')[0].strip()
        else:
            map_id = text.strip()
            map_name = text.strip()
        self.set_now_map(map_id)
        self.show_map(map_id)
        self.relocalization(map_id)

    #设置当前地图 RPC 示例
    def set_now_map(self, map_id=""):
        print("设置当前地图")
        ip_addr = self.ip_input.text().strip() 
        payload = {
            "header": {},
            "command":"MappingCommand_SET_CURRENT_WORKING_MAP",
            "map_id": map_id,
        }
        url = f"http://{ip_addr}:50807/rpc/aimdk.protocol.MappingService/SetCurrentWorkingMap"

        try:
            result = subprocess.run(
                [
                    "curl",
                    "-X",
                    "POST",
                    url,
                    "-H",
                    "Content-Type: application/json",
                    "-d", 
                    json.dumps(payload, ensure_ascii=False),
                ],
                capture_output=True,
                text=True,
                check=True,
            )
            if result.stdout.strip():
                print(f"设置当前地图响应: {result.stdout.strip()}")
                self.show_some.setText(f"设置地图成功")
                # self.get_map_info(map_id)
            else:
                print("未收到设置当前地图响应")
        except FileNotFoundError:
            print("未找到 curl 命令，请安装 curl")
        except subprocess.CalledProcessError as exc:
            err_msg = (exc.stderr or exc.stdout or "未知错误").strip()
            print(f"error: {err_msg}")

    #重定位 RPC 示例
    def relocalization(self, map_id=""):
        print("开始重定位")
        ip_addr = self.ip_input.text().strip() 
        payload = {
            "header": {},
            "command":"RelocalizationCommand_GLOBAL_START",
            "map_id": map_id,
        }
        url = f"http://{ip_addr}:50807/rpc/aimdk.protocol.RelocalizationService/StartGlobalRelocalization"

        try:
            result = subprocess.run(
                [
                    "curl",
                    "-X",
                    "POST",
                    url,
                    "-H",
                    "Content-Type: application/json",
                    "-d", 
                    json.dumps(payload, ensure_ascii=False),
                ],
                capture_output=True,
                text=True,
                check=True,
            )
            if result.stdout.strip():
                print(f"重定位响应: {result.stdout.strip()}")            
        except FileNotFoundError:
            print("未找到 curl 命令，请安装 curl")
        except subprocess.CalledProcessError as exc:
            err_msg = (exc.stderr or exc.stdout or "未知错误").strip()
            print(f"error: {err_msg}")

    def on_connect(self):
        ip_addr = self.ip_input.text().strip()
        if not ip_addr:
            print("IP 地址不能为空")
            return

        if self.ssh_process and self.ssh_process.poll() is None:
            print("已有 SSH 会话正在运行")
            return

        print(f"连接到 {ip_addr}")
        try:
            self.ssh_process = subprocess.Popen(
                [
                    "sshpass", "-p", "1",
                    "ssh",
                    "-o", "StrictHostKeyChecking=no",
                    f"agi@{ip_addr}"
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            # 后台检查连接结果（不阻塞）
            threading.Thread(
                target=self._check_ssh_result,
                args=(self.ssh_process,),
                daemon=True
            ).start()
        except Exception as e:
            print(f"SSH 连接失败: {e}")

    def _check_ssh_result(self, proc: subprocess.Popen):
        try:
            stdout_data, stderr_data = proc.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            print("SSH 连接超时，可能失败")
            self.show_some.setText("SSH 连接超时，可能失败")
            return

        if proc.returncode == 0:
            print("✅ SSH 连接成功")
            self.show_some.setText("✅ SSH 连接成功")
            # 初始化地图列表
            self.get_map_list()
            self.get_now_map()
        else:
            err_msg = (stderr_data or stdout_data or "未知错误").strip()
            print(f"SSH 连接失败: {err_msg}")
            self.show_some.setText(f"SSH 连接失败: {err_msg}")

    # 获取当前地图 RPC 示例
    def get_now_map(self):
        print("获取当前地图")
        ip_addr = self.ip_input.text().strip() 
        payload = {
            "header": {},
            "command":"MappingCommand_GET_CURRENT_WORKING_MAP"
        }
        url = f"http://{ip_addr}:50807/rpc/aimdk.protocol.MappingService/GetCurrentWorkingMap"

        try:
            result = subprocess.run(
                [
                    "curl",
                    "-X",
                    "POST",
                    url,
                    "-H",
                    "Content-Type: application/json",
                    "-d", 
                    json.dumps(payload, ensure_ascii=False),
                ],
                capture_output=True,
                text=True,
                check=True,
            )
            if result.stdout.strip():
                try:
                    resp_json = json.loads(result.stdout)
                    current_map = resp_json.get("data", {}).get("map_id", "")
                    print(f"当前地图: {current_map}")
                    # self.get_map_info(current_map)
                    self.show_map(current_map)
                except json.JSONDecodeError as exc:
                    print(f"解析当前地图失败: {exc}")
            else:
                print("未收到当前地图响应")
        except FileNotFoundError:
            print("未找到 curl 命令，请安装 curl")
        except subprocess.CalledProcessError as exc:
            err_msg = (exc.stderr or exc.stdout or "未知错误").strip()
            print(f"error: {err_msg}")

    # 获取地图信息 RPC 示例
    def show_map(self,map_id=""):
        print("显示地图")
        # self.get_now_map()
        ip_addr = self.ip_input.text().strip() 
        payload = {
            "header": {},
            "command":"MappingCommand_GET_2D_WHOLE_MAP",
            "map_id": map_id,
            "is_map_meta": False,
            "is_http": False,
        }
        url = f"http://{ip_addr}:50807/rpc/aimdk.protocol.MappingService/Get2DWholeMap"

        try:
            result = subprocess.run(
                [
                    "curl",
                    "-X",
                    "POST",
                    url,
                    "-H",
                    "Content-Type: application/json",
                    "-d", 
                    json.dumps(payload, ensure_ascii=False),
                ],
                capture_output=True,
                text=True,
                check=True,
            )
            if result.stdout.strip():
                try:
                    resp_json = json.loads(result.stdout)
                    map_data = resp_json.get("data", {}).get("map_data", "")
                    origin_x = resp_json.get("data", {}).get("origin_x", 0)
                    origin_y = resp_json.get("data", {}).get("origin_y", 0)
                    resolution = resp_json.get('data', {}).get('resolution', 0)
                    map_name = resp_json.get("data", {}).get("map_name", {})
                    image_bytes = base64.b64decode(map_data)
                    self._queue_map_update(image_bytes)
                    #print(f"当前地图: {map_data}")
                    print(f"地图原点x: {origin_x}")
                    print(f"地图原点y: {origin_y}")
                    print(f"地图宽度: {resp_json.get('data', {}).get('width', 0)}")
                    print(f"地图高度: {resp_json.get('data', {}).get('height', 0)}")
                    print(f"地图分辨率: {resolution}")
                    
                    self.show_current_working_map.setText(f"当前地图: {map_name}")
                    # 切换地图时关闭当前定位线程，启动新线程
                    if self.location_thread and self.location_thread.is_alive():
                        self.location_stop.set()
                        self.location_thread.join()
                    self.location_stop.clear()
                    self.location_thread = threading.Thread(
                        target=self.get_now_location,
                        args=(origin_x, origin_y, resolution),
                        daemon=True
                    )
                    self.location_thread.start()

                except json.JSONDecodeError as exc:
                    print(f"解析当前地图失败: {exc}")
            else:
                print("未收到当前地图响应")
        except FileNotFoundError:
            print("未找到 curl 命令，请安装 curl")
        except subprocess.CalledProcessError as exc:
            err_msg = (exc.stderr or exc.stdout or "未知错误").strip()
            print(f"error: {err_msg}")

    def get_now_location(self, origin_x, origin_y, resolution):
        """
        接收里程计数据，实时显示红点。
        origin_x, origin_y: 地图原点（米）
        resolution: 地图分辨率（厘米/像素）
        """
        # 等待连接建立
        while self.conn is None:
            if self.location_stop.is_set():
                return
            time.sleep(0.1)
        while not self.location_stop.is_set():
            try:
                self.conn.settimeout(0.5)
                data = self.conn.recv(4096)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"接收异常: {e}")
                break
            if not data:
                break
            try:
                msg = yaml.safe_load(data.decode())
                x = msg.get("x")
                y = msg.get("y")
                if x is not None and y is not None and resolution:
                    # 米转像素，分辨率为厘米/像素，需先转为米/像素
                    res_m = resolution / 100.0
                    u = int(round(origin_x + x / res_m))
                    v = int(round(origin_y - y / res_m))  # 左上为原点，y向下为正
                    # print(f"x: {x}, y: {y} -> u: {u}, v: {v}")
                    self.show_some.setText(f"位姿: x={x:.2f}m, y={y:.2f}m")
                    # 在主线程安全地更新红点
                    def update_pose():
                        if hasattr(self, "pose_item") and self.pose_item:
                            self.map_scene.removeItem(self.pose_item)
                        self.pose_item = self.map_scene.addEllipse(
                            u - 5, v - 5, 10, 10,
                            QPen(QColor("#ff0000"), 2),
                            QBrush(QColor(255, 0, 0, 180))
                        )
                        self.pose_item.setZValue(10)
                    
                    QApplication.instance().postEvent(self, QEvent(QEvent.User))
                    self._update_pose = update_pose
                else:
                    print(f"无x/y字段或分辨率无效，内容: {msg}")
                    self.show_some.setText("无定位数据或重定位失败")
            except Exception as e:
                print(f"解析失败: {e}, 原始数据: {data.decode()}")

                    
    def event(self, event):
        # 用于主线程安全地更新红点
        if event.type() == QEvent.User and hasattr(self, '_update_pose'):
            self._update_pose()
            del self._update_pose
            return True
        return super().event(event)

    # 获取地图列表 RPC 示例
    def get_map_list(self):
        print("获取地图列表")
        ip_addr = self.ip_input.text().strip() 
        payload = {
            "header": {},
            "command": "MappingCommand_GET_STORED_MAP_NAME"
        }
        url = f"http://{ip_addr}:50807/rpc/aimdk.protocol.MappingService/GetStoredMapNames"

        try:
            result = subprocess.run(
                [
                    "curl",
                    "-X",
                    "POST",
                    url,
                    "-H",
                    "Content-Type: application/json",
                    "-d", 
                    json.dumps(payload, ensure_ascii=False),
                ],
                capture_output=True,
                text=True,
                check=True,
            )
            if result.stdout.strip():
                try:
                    resp_json = json.loads(result.stdout)
                    map_lists = resp_json.get("data", {}).get("map_lists", [])
                    self.map_list.clear()
                    for item in map_lists:
                        map_id = item.get("map_id", "")
                        map_name = item.get("map_name", "")
                        # 显示格式：map_name (map_id)
                        display_text = f"{map_name} ({map_id})" if map_id else map_name
                        self.map_list.addItem(display_text)
                    print(f"地图列表: {map_lists}")
                except json.JSONDecodeError as exc:
                    print(f"解析地图列表失败: {exc}")
            else:
                print("未收到地图列表响应")
        except FileNotFoundError:
            print("未找到 curl 命令，请安装 curl")
        except subprocess.CalledProcessError as exc:
            err_msg = (exc.stderr or exc.stdout or "未知错误").strip()
            print(f"error: {err_msg}")
    
    def on_create_map(self):
        print("创建地图")
        ip_addr = self.ip_input.text().strip() 
        payload = {
            "header": {
                "timestamp": {
                    "seconds": "0",
                    "nanos": 0,
                    "ms_since_epoch": "1744598548952",
                }
            },
            "command": "MappingCommand_STOP_MAPPING",
            "no_realtime_data": "true",
        }
        url = f"http://{ip_addr}:50807/rpc/aimdk.protocol.MappingService/StartMapping"

        try:
            result = subprocess.run(
                [
                    "curl",
                    "-X",
                    "POST",
                    url,
                    "-H",
                    "Content-Type: application/json",
                    "-d", 
                    json.dumps(payload, ensure_ascii=False),
                ],
                capture_output=True,
                text=True,
                check=True,
            )
            msg_text = ""
            if result.stdout.strip():
                print(f"创建地图响应: {result.stdout.strip()}")
                try:
                    resp_json = json.loads(result.stdout)
                    msg_text = resp_json.get("header", {}).get("msg", "")
                except json.JSONDecodeError:
                    msg_text = result.stdout.strip()

            if not msg_text:
                msg_text = "创建地图开始"
            self.show_some.setText(msg_text)
            self._start_map_fetch_loop()
            
        except FileNotFoundError:
            print("未找到 curl 命令，请安装 curl")
            self.show_some.setText("未找到 curl 命令")
        except subprocess.CalledProcessError as exc:
            err_msg = (exc.stderr or exc.stdout or "未知错误").strip()
            print(f"error: {err_msg}")
            self.show_some.setText(err_msg)

    def _start_map_fetch_loop(self):
        if self.map_fetch_thread and self.map_fetch_thread.is_alive():
            return

        self.map_fetch_stop.clear()
        self.map_fetch_thread = threading.Thread(
            target=self._map_fetch_worker,
            daemon=True,
        )
        self.map_fetch_thread.start()

    def _map_fetch_worker(self):
        while not self.map_fetch_stop.is_set():
            try:
                self.get_map()
            except Exception as exc:
                print(f"获取地图循环异常: {exc}")
            time.sleep(2)
   
    # 获取建图信息 RPC 示例
    def get_map(self):
        print("获取地图")
        ip_addr = self.ip_input.text().strip() 
        payload = {
            "header": {
                "timestamp": {
                    "seconds": "0",
                    "nanos": 0,
                    "ms_since_epoch": "1744598548952",
                }
            },
            "command": "MappingCommand_GET_REALTIME_MAP",
        }
        url = f"http://{ip_addr}:50807/rpc/aimdk.protocol.MappingService/GetRealtimeMapData"

        try:
            result = subprocess.run(
                [
                    "curl",
                    "-X",
                    "POST",
                    url,
                    "-H",
                    "Content-Type: application/json",
                    "-d", 
                    json.dumps(payload, ensure_ascii=False),
                ],
                capture_output=True,
                text=True,
                check=True,
            )
            if result.stdout.strip():
                try:
                    resp_json = json.loads(result.stdout)
                    # print(f"建图响应数据: {result.stdout.strip()}")
                except json.JSONDecodeError as exc:
                    print(f"解析地图数据失败: {exc}")
                    self.show_some.setText("解析地图数据失败")
                    return

                map_data = (
                    resp_json
                    .get("data", {})
                    .get("map_data", "")
                )
                pose = resp_json.get("data", {}).get("cur_pos", {})
                if map_data:
                    try:
                        image_bytes = base64.b64decode(map_data)
                        self._queue_map_update(image_bytes)
                        self.show_some.setText("地图数据已获取")
                        print(f"当前位姿: {pose}")
                        # 在地图上显示红点
                        if pose and "position" in pose:
                            pos = pose["position"]
                            u = int(round(pos.get("u", 0)))
                            v = int(round(pos.get("v", 0)))
                            # 清除旧位姿点
                            if hasattr(self, "pose_item") and self.pose_item:
                                self.map_scene.removeItem(self.pose_item)
                            # 添加新位姿点
                            self.pose_item = self.map_scene.addEllipse(
                                u - 5, v - 5, 10, 10,
                                QPen(QColor("#ff0000"), 2),
                                QBrush(QColor(255, 0, 0, 180))
                            )
                            self.pose_item.setZValue(10)
                    except Exception as exc:
                        print(f"地图数据解码失败: {exc}")
                        self.show_some.setText("地图数据解码失败")
                else:
                    self.show_some.setText("未获取到地图数据")
            else:
                self.show_some.setText("未收到地图数据响应")
        except FileNotFoundError:
            print("未找到 curl 命令，请安装 curl")
            self.show_some.setText("未找到 curl 命令")
        except subprocess.CalledProcessError as exc:
            err_msg = (exc.stderr or exc.stdout or "未知错误").strip()
            print(f"error: {err_msg}")
            self.show_some.setText(err_msg)

    def _queue_map_update(self, image_bytes: bytes):
        self.map_data_ready.emit(image_bytes)

    def _update_map_scene(self, image_bytes: bytes):
        pixmap = QPixmap()
        print(f"更新地图场景，图像数据大小: {len(image_bytes)} 字节")
        if pixmap.loadFromData(image_bytes):
            # 保存当前缩放和中心
            old_transform = self.graphicsView.transform()
            old_center = self.graphicsView.mapToScene(self.graphicsView.viewport().rect().center())

            if self.map_pixmap_item is None:
                self.map_pixmap_item = self.map_scene.addPixmap(pixmap)
                self.map_pixmap_item.setZValue(-1) 
            else:
                self.map_pixmap_item.setPixmap(pixmap)

            self.map_scene.setSceneRect(self.map_pixmap_item.boundingRect())
            # fitInView 只在首次加载时调用，后续保持用户缩放
            if old_transform.isIdentity():
                self.graphicsView.fitInView(
                    self.map_pixmap_item.boundingRect(),
                    Qt.KeepAspectRatio,
                )
            else:
                self.graphicsView.setTransform(old_transform)
                # 恢复视图中心
                self.graphicsView.centerOn(old_center)
        else:
            self.show_some.setText("地图图像加载失败")

    # 停止建图 RPC 示例
    def on_cancel_map(self):
        print("取消建图")
        ip_addr = self.ip_input.text().strip() 
        payload = {
            "header": {
                "timestamp": {
                    "seconds": "0",
                    "nanos": 0,
                    "ms_since_epoch": "1744598548952",
                }
            },
            "command": "MappingCommand_STOP_MAPPING",
            # "map_name": "测试地图",
        }
        url = f"http://{ip_addr}:50807/rpc/aimdk.protocol.MappingService/StopMapping"

        try:
            result = subprocess.run(
                [
                    "curl",
                    "-X",
                    "POST",
                    url,
                    "-H",
                    "Content-Type: application/json",
                    "-d", 
                    json.dumps(payload, ensure_ascii=False),
                ],
                capture_output=True,
                text=True,
                check=True,
            )
            msg_text = ""
            if result.stdout.strip():
                print(f"取消建图响应: {result.stdout.strip()}")
                try:
                    resp_json = json.loads(result.stdout)
                    msg_text = resp_json.get("header", {}).get("msg", "")
                except json.JSONDecodeError:
                    msg_text = result.stdout.strip()

            if not msg_text:
                msg_text = "取消建图成功"

            self.show_some.setText(msg_text)
            self._stop_map_fetch_loop()
            # 取消建图后自动刷新当前地图，重启定位线程
            current_item = self.map_list.currentItem()
            if current_item:
                text = current_item.text()
                if '(' in text and ')' in text:
                    map_id = text.split('(')[-1].split(')')[0].strip()
                else:
                    map_id = text.strip()
                self.show_map(map_id)
        except FileNotFoundError:
            print("未找到 curl 命令，请安装 curl")
            self.show_some.setText("未找到 curl 命令")
        except subprocess.CalledProcessError as exc:
            err_msg = (exc.stderr or exc.stdout or "未知错误").strip()
            print(f"停止映射失败: {err_msg}")
            self.show_some.setText(err_msg)

    # 停止建图 RPC 示例
    def on_save_map(self, map_name=""):
        print("保存地图")
        ip_addr = self.ip_input.text().strip() 
        payload = {
            "header": {
                "timestamp": {
                    "seconds": "0",
                    "nanos": 0,
                    "ms_since_epoch": "1744598548952",
                }
            },
            "command": "MappingCommand_SAVING_MAP",
            "map_name": map_name,
        }
        url = f"http://{ip_addr}:50807/rpc/aimdk.protocol.MappingService/StopMapping"

        try:
            result = subprocess.run(
                [
                    "curl",
                    "-X",
                    "POST",
                    url,
                    "-H",
                    "Content-Type: application/json",
                    "-d", 
                    json.dumps(payload, ensure_ascii=False),
                ],
                capture_output=True,
                text=True,
                check=True,
            )
            msg_text = ""
            if result.stdout.strip():
                print(f"保存地图响应: {result.stdout.strip()}")
                self.get_map_list()  # 刷新地图列表
                try:
                    resp_json = json.loads(result.stdout)
                    msg_text = resp_json.get("header", {}).get("msg", "")
                except json.JSONDecodeError:
                    msg_text = result.stdout.strip()

            if not msg_text:
                msg_text = "保存地图成功"

            self.show_some.setText(msg_text)
            self._stop_map_fetch_loop()
        except FileNotFoundError:
            print("未找到 curl 命令，请安装 curl")
            self.show_some.setText("未找到 curl 命令")
        except subprocess.CalledProcessError as exc:
            err_msg = (exc.stderr or exc.stdout or "未知错误").strip()
            print(f"error: {err_msg}")
            self.show_some.setText(err_msg)
        
    def _stop_map_fetch_loop(self):
        if not self.map_fetch_thread:
            return
        self.map_fetch_stop.set()
        self.map_fetch_thread = None

    def on_clear_draw(self):
        """清空所有绘制的点和多边形，恢复初始状态"""
        self._reset_drawing()
        self.show_some.setText("绘制内容已清空")

    def toggle_draw_mode(self):
        self.drawing_enabled = not self.drawing_enabled
        if self.drawing_enabled:
            self._reset_drawing()
            self.show_some.setText("绘制模式：左键打点，右键退出")
            self.graphicsView.setCursor(Qt.CrossCursor)
        else:
            self.show_some.setText("绘制模式已关闭")
            self.graphicsView.setCursor(Qt.ArrowCursor)

    def _reset_drawing(self):
        for item in self.point_items:
            self.map_scene.removeItem(item)
        self.point_items.clear()
        self.drawn_points.clear()
        if self.polygon_item is not None:
            self.map_scene.removeItem(self.polygon_item)
            self.polygon_item = None

    def eventFilter(self, watched, event):
        if watched is self.graphicsView.viewport():
            # 绘图模式下处理鼠标点击
            if self.drawing_enabled:
                if event.type() == QEvent.MouseButtonPress:
                    if event.button() == Qt.LeftButton:
                        # 兼容新旧API，优先用position()
                        pos = event.position().toPoint() if hasattr(event, 'position') else event.pos()
                        self._add_point(pos)
                        return True
                    if event.button() == Qt.RightButton:
                        self.toggle_draw_mode()
                        return True
            # 画布缩放：滚轮事件
            if event.type() == QEvent.Wheel:
                old_pos = self.graphicsView.mapToScene(event.position().toPoint() if hasattr(event, 'position') else event.pos())
                if event.angleDelta().y() > 0:
                    scale_factor = 1.15
                else:
                    scale_factor = 0.85
                current_transform = self.graphicsView.transform()
                current_scale = current_transform.m11()
                min_scale = 0.2
                max_scale = 5.0
                new_scale = current_scale * scale_factor
                if new_scale < min_scale:
                    scale_factor = min_scale / current_scale
                elif new_scale > max_scale:
                    scale_factor = max_scale / current_scale
                self.graphicsView.scale(scale_factor, scale_factor)
                new_pos = self.graphicsView.mapToScene(event.position().toPoint() if hasattr(event, 'position') else event.pos())
                delta = new_pos - old_pos
                self.graphicsView.translate(delta.x(), delta.y())
                return True

            # 鼠标中键拖动画布
            if event.type() == QEvent.MouseButtonPress:
                if event.button() == Qt.MiddleButton:
                    self._panning = True
                    self._pan_start = event.position().toPoint() if hasattr(event, 'position') else event.pos()
                    self.graphicsView.setCursor(Qt.ClosedHandCursor)
                    return True
            elif event.type() == QEvent.MouseMove:
                if getattr(self, '_panning', False):
                    # 计算移动距离
                    cur_pos = event.position().toPoint() if hasattr(event, 'position') else event.pos()
                    delta = cur_pos - self._pan_start
                    self._pan_start = cur_pos
                    self.graphicsView.horizontalScrollBar().setValue(
                        self.graphicsView.horizontalScrollBar().value() - delta.x())
                    self.graphicsView.verticalScrollBar().setValue(
                        self.graphicsView.verticalScrollBar().value() - delta.y())
                    return True
            elif event.type() == QEvent.MouseButtonRelease:
                if event.button() == Qt.MiddleButton and getattr(self, '_panning', False):
                    self._panning = False
                    self._pan_start = None
                    self.graphicsView.setCursor(Qt.ArrowCursor)
                    return True
        return super().eventFilter(watched, event)

    def _add_point(self, widget_pos):
        scene_pos = self.graphicsView.mapToScene(widget_pos)
        self.drawn_points.append(scene_pos)

        point_item = self.map_scene.addEllipse(
            scene_pos.x() - 3,
            scene_pos.y() - 3,
            6,
            6,
            self.point_pen,
            self.point_brush,
        )
        point_item.setZValue(5)
        self.point_items.append(point_item)

        self._update_polygon()

    def _update_polygon(self):
        if len(self.drawn_points) >= 3:
            points = list(self.drawn_points)
            if points[0] != points[-1]:
                points.append(points[0])
            polygon = QPolygonF(points)
            if self.polygon_item is None:
                self.polygon_item = self.map_scene.addPolygon(
                    polygon,
                    self.polygon_pen,
                    self.polygon_brush,
                )
                self.polygon_item.setZValue(4)
            else:
                self.polygon_item.setPolygon(polygon)
        elif self.polygon_item is not None:
            self.map_scene.removeItem(self.polygon_item)
            self.polygon_item = None

    def on_unknow_area(self):
        # 收集当前多边形点，转换为后端需要的vertices格式
        if len(self.drawn_points) < 3:
            self.show_some.setText("请先绘制多边形区域")
            return
        # 计算每个点的线段朝向（指向下一个点）
        points = self.drawn_points
        n = len(points)
        vertices = []
        for i in range(n):
            p1 = points[i]
            p2 = points[(i+1)%n]  # 闭合多边形
            angle = self._calc_angle(p1, p2)
            # 转换为像素整数
            vertices.append({
                "position": {"u": int(round(p1.x())), "v": int(round(p1.y()))},
                "angle": angle
            })
        # 取当前地图id
        item = self.map_list.currentItem()
        map_id = ""
        if item and '(' in item.text() and ')' in item.text():
            map_id = item.text().split('(')[-1].split(')')[0].strip()
        # color和type可根据需求调整
        region = {
            "vertices": vertices,
            "color": "FFFFFFFF",#颜色
            "type": "VerteciesType_CLOSURE"
        }
        self.on_SyncRegion(map_id, [region])

    def _calc_angle(self, p1: QPointF, p2: QPointF):
        """
        计算从p1指向p2的线段朝向角度（弧度，逆时针，0为u轴正方向）。
        """
        dx = p2.x() - p1.x()
        dy = p2.y() - p1.y()
        return math.atan2(dy, dx)

    #修改底图接口示例
    def on_SyncRegion(self, map_id="", regions=None):
        print("同步区域")
        ip_addr = self.ip_input.text().strip()
        if regions is None:
            regions = []
        payload = {
            "header": {
                "timestamp": {
                    "seconds": "0",
                    "nanos": 0,
                    "ms_since_epoch": "1744598548952"
                }
            },
            "command": "MappingCommand_SYNC_REGION",
            "map_id": map_id,
            "regions": regions
        }
        url = f"http://{ip_addr}:50807/rpc/aimdk.protocol.MappingService/SyncRegion"
        print("SyncRegion req url:", url)
        print("SyncRegion req payload:", json.dumps(payload, ensure_ascii=False))
        try:
            result = subprocess.run(
                [
                    "curl",
                    "-X",
                    "POST",
                    url,
                    "-H",
                    "Content-Type: application/json",
                    "-d",
                    json.dumps(payload, ensure_ascii=False),
                ],
                capture_output=True,
                text=True,
                check=True,
            )
            if result.stdout.strip():
                print(f"同步区域响应: {result.stdout.strip()}")
                self.get_now_map()  # 同步区域后刷新地图显示
                self.on_clear_draw ()  # 清除绘制状态
            else:
                print("未收到同步区域响应")
        except FileNotFoundError:
            print("未找到 curl 命令，请安装 curl")
        except subprocess.CalledProcessError as exc:
            err_msg = (exc.stderr or exc.stdout or "未知错误").strip()
            print(f"error: {err_msg}")

    def on_delete_map_dialog(self):
        from PySide6.QtWidgets import QMessageBox
        item = self.map_list.currentItem()
        if item is None:
            self.show_some.setText("请先选择一个地图")
            return
        text = item.text()
        if '(' in text and ')' in text:
            map_id = text.split('(')[-1].split(')')[0].strip()
        else:
            self.show_some.setText("无法获取地图ID，无法删除")
            return
        reply = QMessageBox.question(self, "确认删除", f"确定要删除该地图吗？\nID: {map_id}",
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            self._delete_map(map_id)
        else:
            self.show_some.setText("已取消删除")
        

    def on_delete_map(self):
        item = self.map_list.currentItem()
        if item is None:
            self.show_some.setText("请先选择一个地图")
            return
        text = item.text()
        if '(' in text and ')' in text:
            map_id = text.split('(')[-1].split(')')[0].strip()
            self._delete_map(map_id)
        else:
            self.show_some.setText("无法获取地图ID，无法删除")
        
    #删除地图接口示例
    def _delete_map(self,map_id=""):
        print("删除地图")
        ip_addr = self.ip_input.text().strip() 
        payload = {
            "header": {},
            "command": "MappingCommand_DELETE_MAP",
            "map_id": map_id, 
        }
        url = f"http://{ip_addr}:50807/rpc/aimdk.protocol.MappingService/DeleteMap"

        try:
            result = subprocess.run(
                [
                    "curl",
                    "-X",
                    "POST",
                    url,
                    "-H",
                    "Content-Type: application/json",
                    "-d", 
                    json.dumps(payload, ensure_ascii=False),
                ],
                capture_output=True,
                text=True,
                check=True,
            )
            if result.stdout.strip():
                print(f"删除地图响应: {result.stdout.strip()}")
                self.get_map_list()  # 刷新地图列表
            else:
                print("未收到删除地图响应")
        except FileNotFoundError:
            print("未找到 curl 命令，请安装 curl")
        except subprocess.CalledProcessError as exc:
            err_msg = (exc.stderr or exc.stdout or "未知错误").strip()
            print(f"error: {err_msg}")

    def on_rename_map(self):
        from PySide6.QtWidgets import QInputDialog, QMessageBox
        item = self.map_list.currentItem()
        if item is None:
            QMessageBox.warning(self, "未选中地图", "请先选择一个地图进行重命名！")
            return
        text = item.text()
        if '(' in text and ')' in text:
            map_id = text.split('(')[-1].split(')')[0].strip()
            old_name = text.split('(')[0].strip()
            new_name, ok = QInputDialog.getText(self, "重命名地图", "请输入新的地图名称:", text=old_name)
            if ok and new_name.strip():
                self.rename_map(map_id, old_name, new_name.strip())
        else:
            QMessageBox.warning(self, "地图ID错误", "无法获取地图ID，无法重命名！")
        
    #重命名地图接口示例    
    def rename_map(self,map_id="", old_name="", new_name=""):
        print("重命名地图")
        ip_addr = self.ip_input.text().strip() 
        payload = {
            "header": {},
            "command": "MappingCommand_RENAME_MAP",
            "map_id": map_id, 
            "old_name": old_name,
            "new_name": new_name,
        }
        url = f"http://{ip_addr}:50807/rpc/aimdk.protocol.MappingService/RenameMap"

        try:
            result = subprocess.run(
                [
                    "curl",
                    "-X",
                    "POST",
                    url,
                    "-H",
                    "Content-Type: application/json",
                    "-d", 
                    json.dumps(payload, ensure_ascii=False),
                ],
                capture_output=True,
                text=True,
                check=True,
            )
            if result.stdout.strip():
                print(f"重命名地图响应: {result.stdout.strip()}")
                self.get_map_list()  # 刷新地图列表
            else:
                print("未收到重命名地图响应")
        except FileNotFoundError:
            print("未找到 curl 命令，请安装 curl")
        except subprocess.CalledProcessError as exc:
            err_msg = (exc.stderr or exc.stdout or "未知错误").strip()
            print(f"error: {err_msg}")

if __name__ == "__main__":
    app = QApplication([])
    win = MapDialog()
    win.show()
    app.exec()
