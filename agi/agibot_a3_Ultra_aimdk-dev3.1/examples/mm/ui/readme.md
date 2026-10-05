## 🗺️ 地图工具使用说明

本工具是基于 PySide6 开发的地图编辑与管理示例程序，主要用于演示地图相关 API 的基本用法，适合开发者学习和参考,不可直接用于生产环境。

主要文件说明：
1. map.ui：QT Designer 设计的界面源文件，可用 QT Designer 修改和保存。
2. main.py：核心逻辑代码，展示地图管理、编辑、通信等 API 的实际调用方式。
3. map_ui.py：由 map.ui 转换生成的 Python UI 文件，负责界面控件的初始化。
4. odometry.py：用于转发机器人上的ros2话题

注意事项：
- 本工具仅用于功能演示和开发参考，不可直接用于生产环境。
- 如遇到 BUG 或有改进建议，请及时反馈。
- 用户可根据自身需求扩展功能。

欢迎开发者学习、交流和二次开发。

---

## 📦 环境准备

```bash
#开发环境ubuntu22.04 Python 3.10.12
#复制examples/mm/ui 到与机器人同一局域网下的linux电脑
pip install uv ##安装uv
#安装uv后再运行
uv venv
uv sync

#如遇rpc接口访问被拒执行如下命令
# hdu上执行：
sudo iptables -t nat -A PREROUTING -p tcp --dport 50807 -j DNAT --to-destination 10.42.10.11:50807
sudo iptables -t nat -A POSTROUTING -p tcp -d 10.42.10.11 --dport 50807 -j MASQUERADE
```

---

## 🚀 启动程序

### Step 1： 运行主程序

```bash
uv run python main.py
```

---

### Step 2： 测试通信是否成功

1. 在 **IP 输入栏** 填写机器人局域网 IP
2. 点击 **连接测试**
3. 若地图列表出现数据，说明通信成功

**IP 查询命令：**

```bash
ifconfig
```

---

### Step 3： 机器人端 ROS 话题转发

在机器人x86上执行：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=232
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/software/v0/entry/bin/cfg/ros_dds_configuration.xml
ros2 topic list | grep /slam/localization/odometry #验证话题是否存在

cd examples/mm/ui
python3 odometry.py ##必须启动 `odometry.py`，否则 UI 无法接收定位消息。
```
---

### 🔔 话题说明

* `odometry.py` 转发话题/slam/localization/odometry,该话题为导航定位使用的位姿信息，建图使用的定位信息 **不是** 该话题
* 建图定位信息在RPC接口GetRealtimeMapData,/slam/mapping/odometry话题内也是建图定位信息,用户可根据需求使用，本例程采用RPC接口数据展示建图定位

---

# 🧭 功能按钮说明

---

## 🗺️ 创建地图 / 保存建图 / 取消建图

用于演示 **开始建图 / 停止建图 RPC 接口**。

### 使用流程

1. 点击 **创建地图** → 开始建图
2. 建图完毕后必须执行以下任意按钮结束建图：

   * 保存建图
   * 取消建图

> ⚠️ 若未结束建图，会影响其他功能运行

---

## 绘制模式 / 未知区域 / 清空绘画

用于演示 **地图修改 RPC 接口**。

### 当前 UI 已实现

* 多边形+地图未知区域添加

### 需要用户自行实现

* 已知区域
* 特征边界
* 折线修改
>⚠️ 注意折线修改仅在特征边界添加时生效

---

### 如何修改地图

1. 点击 **绘制模式**
2. 在地图上打点：
   * 左键：打点
   * 右键：退出绘制
3. 点数要求：**≥ 3 个**
4. 必须按多边形顶点顺序打点，顺序错误会导致多边形异常
5. 点击 **未知区域** 即可添加未知区域部分到地图上
6. 点击 **清空绘画** 可清理绘制模式打的点（仅清除ui上的画布内容，无法清理已经对地图做的修改）
7. 用户可参考绘制模式/未知区域/清空绘画三者的代码，完成其他地图修改的功能实现
---


## 重命名地图

演示 **重命名地图 RPC 接口**

---

## 切换地图

演示 **设置当前工作地图 RPC 接口**

---

## 删除地图

演示 **删除地图 RPC 接口**

---

## 重定位

演示 **重定位 RPC 接口**

* 切换地图会自动触发一次重定位
* 若切换地图后定位失败,需手动调用 **重定位**
* 若无法接收到定位信息，请重启odometry.py

---

## 刷新列表

演示 **获取地图列表 RPC 接口**

---

# UI 编译

如需修改页面，用户可使用QT编辑map.ui，再执行如下指令即可生成新的ui：

```bash
uv run python trans_ui2py.py map.ui map_ui.py 
```

---

