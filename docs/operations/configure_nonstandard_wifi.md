# 配置泰芯非标准 Wi-Fi（Wi-Fi HaLow）AP

Status: Validated（2026-08-24 已通过 AP→STA→桥接→静态 IPv4 双向连通验证；不包含机器人动作测试）

本文把 `20260824-021539.png`、`20260824-020804.png`、`20260824-020820.png` 中能辨认的命令整理成可执行流程，
并说明本机工作区 `linux_netat.cpp` 应该改哪里。这里的
MDU（机器人侧 ARM64 计算单元）只涉及网络设置；本文不授权任何关节或
机器人运动命令。

原始材料是本机未版本化的三张现场截图和 `linux_netat.cpp`；本提交只保存逐项转录、现场回执和操作
结论，不把这些用户输入资产或编译产物纳入 Git。

## 先看结论

- 所谓“把第 259 行改成 AP 网口名称”，是把 `linux_netat.cpp` 中传给 `netat_init_c()` 的
  **电脑有线网卡接口名**改成连接泰芯 AP/网桥的接口名，不是填 SSID、IP 地址或无线网卡名。
- 当前仓库这份源码的第 259 行是 `break;`；网口设置实际位于第 356–359 行附近。不要依赖旧手册的
  行号，搜索 `ipstr =`。
- 2026-08-24 在本机看到的有线接口是 `enp129s0`，地址为 `192.168.100.112/24`。插拔网线后仍须
  重新确认，不能把这个名字当成其他电脑的固定值。
- 新补充的原始截图要求在编译前手动给该有线口设置静态 IP，示例为 `192.168.120.120`；本次按现场
  设计采用 `192.168.100.112/24`，并已完成双向验证，因此不应照抄截图示例。
- 本机源码当前连续写了 `enp0s31f6` 和 `enp129s0` 两次；后一次会覆盖前一次并已成功发现 AP。后续
  清理时应合并成单一赋值，但该用户源码不随本次文档提交入库。
- 程序通过指定接口在 UDP `56789` 端口广播发现设备，随后把终端中输入的 `AT+...` 行发给当前选中的
  泰芯设备。

## 1. 接线与确认有线网口

用网线直接连接电脑与要配置的泰芯 AP/网桥。先执行：

```bash
ip -br link
ip -br addr
```

如果不确定哪个是 AP 网口，可先拔掉网线执行一次 `ip -br link`，插回后再执行一次；状态随插拔变化的
有线接口就是候选。接口通常以 `enp...` 或 `eth...` 开头。本机当前候选为：

```text
enp129s0  UP  192.168.100.112/24
```

不要选 `lo`、`docker0`、VPN/TUN 接口，也不要因为 AP 最终提供无线网络而选择电脑的 `wlp...`
无线接口。

原始截图在“将 AP 进行配置”之前还要求：若接入电脑，需要手动设置网口 IP，参数按现场网络配置，
示例为：

```text
IP 地址：192.168.120.120
```

因此应在系统网络设置中把 `enp129s0` 对应的有线连接改为手动 IPv4，并按 AP/现场手册填写地址、
掩码等参数。截图只明确给出了示例 IP，没有拍到掩码、网关和 DNS 的确切值；不要擅自填写未知值。
设置后至少确认：

```bash
ip -br addr show enp129s0
```

本次现场已选择 `192.168.100.0/24` 子网，电脑保持 `192.168.100.112/24`；不采用截图中的
`192.168.120.120` 示例。后续机器人和其他控制端必须使用 `192.168.100.x/24` 中互不冲突的地址。
分配机器人地址前先核对现有设备清单，不能仅凭本文猜一个空闲地址。

## 2. 修改 `linux_netat.cpp`

在仓库根目录定位实际代码：

```bash
cd /home/agiuser/projects/nohope
rg -n 'ipstr\s*=' linux_netat.cpp
```

把当前的：

```cpp
char *ipstr;

ipstr = "enp0s31f6";
ipstr = "enp129s0";
```

改成一个可写字符数组；若第 1 步确认的接口不是 `enp129s0`，替换成实际名称：

```cpp
char ipstr[] = "enp129s0";
```

这样既避免第二次赋值覆盖第一次，也避免把字符串常量赋给 `char *` 的编译警告。不要改第 259 行的
`break;`。

## 3. 编译并启动

现场补充确认的完整编译并运行命令是：

```bash
cd /home/agiuser/projects/nohope
g++ linux_netat.cpp -o linux_netat.o && ./linux_netat.o
```

`linux_netat.o` 虽然使用 `.o` 后缀，但这里没有 `-c`，所以 `g++` 会完成链接并生成可执行文件；
`&&` 保证只有编译成功才启动程序。不要把它误写成只生成目标文件的：

```bash
g++ -c linux_netat.cpp -o linux_netat.o
```

源码若仍保留两个旧网口名，编译可能只有警告，但启动会在实际生效的旧接口上报
`SO_BINDTODEVICE ... No such device`。必须先完成第 2 节修改，再执行上述编译运行命令。

正常时先看到：

```text
init success
```

发现一个设备后还会显示类似：

```text
----- Discover 1 Device -----
device: 1: xx:xx:xx:xx:xx:xx
auto select device 1
```

如果发现多个设备，先在提示符输入设备编号（例如 `1`）并回车，再输入 AT 指令。若
`SO_BINDTODEVICE` 返回 `Operation not permitted`，确认源码和二进制可信后才可分开编译并用权限运行：

```bash
g++ linux_netat.cpp -o linux_netat.o
sudo ./linux_netat.o
```

按 `Ctrl-C` 退出。启动程序本身会广播扫描；真正改变 AP 配置的是后续输入的 AT 指令。

## 4. 截图一中的代码提取

### 4.1 先做只读基线查询

2026-08-24 现场使用 `enp129s0` 启动成功，输出 `init success`，并发现、自动选择一个 MAC 为
`ea:5c:43:76:5a:c0` 的设备。发现成功只证明电脑到 AP 的发现链路可用，不证明固件命令集或配置正确。
发送任何设置前，先在 `>` 提示符逐条查询并保存响应：

```text
at+version
at+wifimode=?
at+ssid=?
at+bss_bw=?
at+chan_list=?
at+encrypt=?
at+psk=?
at+key=?
```

`AT+PSK` 与 `AT+KEY` 可能属于不同固件流程；完成查询前不要把两套密码命令都写入设备。密码查询还
可能回显敏感内容，不要把真实密码提交到 Git 或公开日志。

2026-08-24 现场基线已取得：

| 项目 | 查询结果 |
| --- | --- |
| 固件 | `v2.4.1.3-41928, app:0` |
| 模式 | `ap` |
| SSID | `SJTU-RL2-LAB` |
| BSS 带宽 | `8 MHz` |
| 信道列表 | `9080,9160,9240` |
| 加密 | `1`，已开启 |
| `AT+PSK=?` | 支持；分行返回 32 字节值，本文不记录其内容 |
| `AT+KEY=?` | 支持；会回显明文密码，本文不记录其内容 |

这组结果已经与截图要求的 AP 模式、8 MHz、三频点和开启加密一致。如果 `SJTU-RL2-LAB` 是目标
SSID，则不要重复发送设置，按 `Ctrl-C` 退出 `linux_netat.o` 并进入机器人侧配置。现场查询日志曾
回显并对外粘贴明文密码，因此该密码应视为已泄露；如果网络不是一次性隔离测试网，应在确认 AP、
机器人和遥控器的统一改密/恢复流程后轮换，不能只改 AP 一端。

### 4.2 截图命令转录

截图中可确认的内容如下。原截图按此顺序显示；`xxxxxx` 是用户自定义内容：

```text
at+wifimode=ap

# 下面的 xxxxxx 改成你想要设置的 SSID
at+ssid=xxxxxx

at+bss_bw=8

at+chan_list=9080,9160,9240

at+encrypt=1

# sha256 表示对密码 xxxxx 进行 SHA-256 计算
at+psk=<密码的 SHA-256 十六进制摘要>
```

其中截图只显示到 `at+chan_list=9080,9160,92...`；完整值
`9080,9160,9240` 已与泰芯官方 V2.x AT 指令文档的查询响应和初始化示例交叉核对。数值是中心频点
MHz 的十倍，例如 `9080` 表示 `908 MHz`。AP 与接入它的 STA 必须使用完全相同、顺序也相同的
信道列表。

截图的密码注释和网址右侧也被裁掉。无需把密码交给第三方网站，可在另一个终端本地生成无换行的
SHA-256 摘要：

```bash
read -rsp 'HaLow password: ' HALOW_WIFI_PASSWORD; echo
printf '%s' "${HALOW_WIFI_PASSWORD}" | sha256sum
unset HALOW_WIFI_PASSWORD
```

把输出第一列的 64 个十六进制字符填入 `at+psk=`，不要把占位符、尖括号或明文密码原样输入。

## 5. 实际输入顺序与固件差异

泰芯当前官方 V2.x 文档建议先设置网络参数，最后设置工作模式。因此在 `netat` 提示符中，推荐逐行输入
并确认每条都返回 `OK`：

```text
at+ssid=<你的 SSID>
at+bss_bw=8
at+chan_list=9080,9160,9240
at+encrypt=1
at+psk=<64 位 SHA-256 十六进制摘要>
at+wifimode=ap
```

注意不同泰芯固件的密码接口可能不同：

- 随截图交付的旧流程使用 `AT+PSK=<SHA-256 摘要>`；按截图配置时使用这一种。
- 当前公开 V2.x 文档使用 `AT+KEY=<至少 8 个 ASCII 字符的密码>`，不使用预先计算的摘要。
- 不要在没有确认固件/命令集的情况下把 `AT+PSK` 和 `AT+KEY` 都写一遍。本次
  `v2.4.1.3-41928` 查询确认两者均存在：`PSK` 返回 32 字节值，`KEY` 直接回显明文密码。查询支持
  不代表两条设置命令应同时发送；如需改密，以该固件随附手册规定的单一设置入口为准。

完成后用查询命令核对，不要只凭终端没有报错来判断：

```text
at+wifimode=?
at+ssid=?
at+bss_bw=?
at+chan_list=?
at+encrypt=?
at+channel=?
```

密码查询可能回显敏感内容，不建议在录屏、共享终端或会持久化完整日志的环境中查询。

## 6. 频段、带宽、频点列表和当前 channel

这四个概念不能混用：

- **硬件/地区频段**：模块射频前端和当地法规共同允许的整体范围。当前配置
  `9080,9160,9240 + 8 MHz` 覆盖 `904–928 MHz`，属于 `902–928 MHz` 这一类配置。
- **`BSS_BW`**：每个信道的占用带宽，只允许 `1/2/4/8 MHz`。
- **`CHAN_LIST`**：可选中心频点列表，单位是 `0.1 MHz`；例如 `9080` 表示 `908 MHz`。最多 16 个，
  官方建议不要为扫描速度设置过多。
- **`CHANNEL`**：AP 实际选择 `CHAN_LIST` 中第几个条目，序号从 `1` 开始。

当前列表的映射是：

| `CHANNEL` | 中心频点 | 8 MHz 占用范围 |
| --- | --- | --- |
| `1` | `908 MHz` | 约 `904–912 MHz` |
| `2` | `916 MHz` | 约 `912–920 MHz` |
| `3` | `924 MHz` | 约 `920–928 MHz` |

先只读查询当前选择：

```text
at+channel=?
```

若只想在当前 8 MHz 三个信道中切换，不改列表或带宽，可在 AP 上设置列表序号。例如切到
`916 MHz`：

```text
at+channel=2
at+channel=?
```

该设置会使已连接 STA 发生瞬时断链/重连；机器人尚未接入时风险较小，接入后必须安排维护窗口。

### 改变带宽和可选中心频点

泰芯给出的 `902–928 MHz` 参考组合如下；只在硬件型号和现场无线许可明确覆盖该范围时使用：

| 带宽 | 中心频点列表 |
| --- | --- |
| `8 MHz` | `9080,9160,9240` |
| `4 MHz` | `9060,9100,9140,9180,9220,9260` |
| `2 MHz` | `9050,9070,9090,9110,9130,9150,9170,9190,9210,9230,9250` |
| `1 MHz` | `9035` 到 `9265`，每隔 `10`（即 1 MHz）；列表最多 16 个，应只选需要的非重叠子集 |

例如改成 4 MHz 六信道，并让 AP 使用 `918 MHz`（列表第 4 项）：

```text
at+bss_bw=4
at+chan_list=9060,9100,9140,9180,9220,9260
at+channel=4
at+bss_bw=?
at+chan_list=?
at+channel=?
```

修改后，所有 STA/机器人侧必须使用相同的 `BSS_BW` 和完全相同、顺序也相同的 `CHAN_LIST`，否则
无法连接。`CHANNEL` 通常由 AP 选择；STA 按同一列表扫描。

不要为了切换正常网络信道而执行 `AT+TEST_START`、`AT+LO_FREQ`、`AT+TX_START` 或连续发射命令；
这些属于射频认证/测试模式，其中 `LO_FREQ` 还明确不做范围检查。也不要把模块从 `902–928 MHz`
直接改到 `863–868 MHz`、日本 `920 MHz` 子带或其他地区频段，除非模块具体料号、天线/前端、固件和
当地无线许可均已确认。

官方参考：[AT 指令说明](https://taixin-semi.com/zh/docs/txw8301/latest/%E6%B3%B0%E8%8A%AFah-sdk-v2-x-at%E6%8C%87%E4%BB%A4%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E-v1-3)、
[802.11ah 频点设置说明](https://taixin-semi.com/en/docs/txw8301/latest/%E6%B3%B0%E8%8A%AF802-11ah%E9%A2%91%E7%82%B9%E8%AE%BE%E7%BD%AE%E8%AF%B4%E6%98%8E-v2-3)。

## 7. 截图二：机器人侧配置

截图中的文字步骤为：

1. 遥控器与机器人都使用 3.1 版本。
2. 在遥控器中修改“天罡配置信息”，把开关切换到“群控”，填写网络配置；机器人 IP 必须与 AP
   位于同一网段。保存后遥控器可能无法继续连接。需要恢复遥控器接管时，重启机器人，或使用 PC 版
   Aim Master 切回遥控模式。
3. 保存后 SSH 登录机器人 MDU。
4. 在 MDU 执行网络参数命令。
5. 检查网络是否通畅。

### 本分支选定的群控填写值（2026-08-24）

现场选择 `192.168.100.0/24`，并把截图示例的 `.128` 机器人地址和 `.10` 网关角色等位迁移到该
子网。群控界面直接填写：

| 字段 | 填写值 | 含义 |
| --- | --- | --- |
| 控制模式 | `群控` | 切换后原遥控连接可能断开 |
| IP | `192.168.100.128` | 本次为机器人分配的静态地址 |
| 子网掩码 | `255.255.255.0` | 即 `/24` |
| 网关 | `192.168.100.10` | 按原截图 `.10` 角色迁移的群控网关地址 |
| 接入点名称 | `A3Test` | 用户选定的目标名称；AP 当前仍需从 `SJTU-RL2-LAB` 改为此值 |

电脑有线口保持：

```text
enp129s0 = 192.168.100.112/24
```

填写前已从该电脑分别向 `192.168.100.128` 和 `192.168.100.10` 发送三次 ICMP 探测；两者均无响应，
邻居表也未解析到 MAC，因此当前在线链路未发现地址冲突。但离线设备不会响应，此检查不能替代团队
地址分配表。`.10` 当前也未响应，所以这里只把它定义为按手册模板选定的群控网关填写值，不能声称
已有可达路由器。

保存前再次核对五个字段；保存后按截图提示，遥控器可能无法继续连接。必须保留网线/SSH 或 PC 版
Aim Master 的恢复路径。不要同时把电脑和机器人设置成 `.128`，也不要把网络地址 `.0` 或广播地址
`.255` 分配给主机。

群控保存前，先在 `linux_netat.o` 中把 AP 的 SSID 改为设计值并查询确认：

```text
at+ssid=A3Test
at+ssid=?
```

必须看到 `SSID: A3Test` 和 `OK` 才能继续。SSID 区分大小写，群控界面也必须精确填写 `A3Test`。
本操作只改接入点名称，不主动改现有加密开关或密码；该固件会在 `AT+KEY=?` 中回显明文密码，不要在
共享日志中再次查询或粘贴。SSID 变更可能让已连接的 STA 断开，因此应先改 AP、确认，再保存机器人
群控配置。

截图代码框右侧被裁掉，只能逐字确认到：

```text
sudo hgpriv taixin_mdu set b...
sudo hgpriv taixin_mdu set c...
```

结合截图一的参数名，最可能的完整命令是下面两条，但这部分属于**补全推断，不是完整 OCR**：

```bash
sudo hgpriv taixin_mdu set bss_bw=8
sudo hgpriv taixin_mdu set chan_list=9080,9160,9240
```

在 MDU 上真正执行前，必须先用设备自带的 `hgpriv` 帮助/查询功能确认 `taixin_mdu` 接口存在，并确认
参数名确实为 `bss_bw` 和 `chan_list`。如果帮助中使用 `freq_range` 等不同参数，不要强行执行上述补全。
AP 与 MDU 的带宽、频点列表必须一致。

网络连通性检查应使用现场实际地址，例如：

```bash
ip -br addr show taixin_mdu
ping -c 4 "AP_OR_PEER_IP"  # replace with the actual AP or peer IPv4 address
```

截图第 5 步的代码框未拍全，无法从现有图片确认它原本指定的目标 IP。

## 8. 失败时先查这些项目

- **编译成功但发现不了设备**：确认选的是直连 AP 的有线接口；检查网线、接口 `UP` 状态和是否存在
  多个隔离网段；临时关闭防火墙前先按现场安全策略确认，不要直接清空所有防火墙规则。
- **显示多个设备**：输入正确的设备编号后再发 AT 指令，避免改错网桥。
- **AT 指令返回 `ERROR`**：先查固件版本和该版本支持的命令，尤其不要混用 `PSK` 与 `KEY`。
- **AP 与机器人互相看不到**：确认两端 `bss_bw`、`chan_list`（包括顺序）、SSID、加密方式和密码
  完全一致，并确认 IP/掩码处于同一子网。
- **配置后遥控器掉线**：这是截图明确提示的预期风险；确保改动前已有网线或 PC Aim Master 的恢复路径。
- **重启后参数丢失**：逐项重新查询。不同固件对掉电保存的支持不同，不能仅凭一次 `OK` 假定已持久化。

### 群控保存后 `192.168.100.128` ping 不通（2026-08-24 现场诊断）

现场保存机器人 `.128/24`、网关 `.10`、接入点 `A3Test` 后，从电脑 `.112/24` ping 机器人失败。
只读诊断结果为：

```text
route: 192.168.100.128 dev enp129s0 src 192.168.100.112
ping: Destination Host Unreachable
neighbor: 192.168.100.128 INCOMPLETE
```

这说明路由和电脑地址正确，但电脑无法通过 ARP 解析机器人 MAC；故障在同子网二层可达性之前，不能
先归因于 ICMP 防火墙，也不应通过修改网关来修。按以下顺序排查：

1. 在仍运行的 `linux_netat.o` 中查询 AP 当前值：

   ```text
   at+ssid=?
   at+bss_bw=?
   at+chan_list=?
   at+channel=?
   at+syscfg
   ```

   必须确认 SSID 已持久化为 `A3Test`，带宽为 `8`，列表为 `9080,9160,9240`，并记录 AP 当前
   `CHANNEL`。不要在这一步再次查询/粘贴明文 `KEY`。

   现场已得到该回执：`SSID=A3Test`、`BSS_BW=8`、
   `CHAN_LIST=9080,9160,9240`、`CHANNEL=1`（中心频点 `908 MHz`），全部返回 `OK`；
   `AT+SYSCFG` 未显示回显。四项独立查询已经排除 AP 当前配置错误，后续排查转到机器人侧。

2. 在机器人本地屏幕/遥控器重新打开群控页面，确认保存后仍显示 `.128`、`255.255.255.0`、`.10`、
   `A3Test`。仅输入但未成功保存不算通过。
3. 使用不依赖该无线链路的维护路径（机器人有线口、现场终端或 PC Aim Master）登录 MDU。先查看
   `taixin_mdu` 是否存在及当前驱动参数；不要因为无线不通而从同一无线地址尝试 SSH。
4. 截图右侧裁边推断的两条设置为：

   ```bash
   sudo hgpriv taixin_mdu set bss_bw=8
   sudo hgpriv taixin_mdu set chan_list=9080,9160,9240
   ```

   必须先用 MDU 本机 `hgpriv` 帮助或只读输出确认参数名；执行后要保存完整回显。AP 与机器人 STA 的
   `bss_bw`、`chan_list` 及顺序必须完全一致。
5. 按机器人/Aim Master 支持的流程重启网络或机器人，使群控与驱动设置生效；不要在承重/运动状态下
   随意重启服务。重新检查机器人本地 `taixin_mdu` 地址是否为 `.128/24`，再从电脑执行：

   ```bash
   ping -c 4 192.168.100.128
   ip neigh show 192.168.100.128 dev enp129s0
   ```

只有邻居状态从 `INCOMPLETE` 变为带 MAC 的可达状态，才说明二层链路建立。网关 `.10` 对同一 `/24`
内的 `.112 ↔ .128` 通信不参与转发，因此 `.10` 不可达不是本次同网段 ping 失败的直接原因。

### Conference Hall 与 A3Test 是两条不同链路

现场确认机器人当前可见地连接在普通 Wi-Fi `Conference Hall`，电脑则同时具有：

```text
wlp130s0f0 -> Conference Hall -> 10.10.60.121/24
enp129s0   -> 有线直连泰芯 HaLow AP A3Test -> 192.168.100.112/24
```

目标机器人应另外具有：

```text
普通 Wi-Fi（可以继续连接 Conference Hall）
taixin_mdu HaLow STA -> A3Test -> 192.168.100.128/24
```

普通 Wi-Fi 与 HaLow 接口可以并存；“机器人仍连着 Conference Hall”本身不能证明错误，也不要求先断开
Conference Hall。必须在机器人 MDU 本地检查第二个接口：

仓库记录的维护拓扑是 laptop → HDU `172.23.20.135` → MDU `10.42.10.12`。从电脑经 HDU 跳板登录
MDU：

```bash
ssh -tt -J agi@172.23.20.135 agi@10.42.10.12
```

如果已经登录 HDU，则直接执行：

```bash
ssh -tt agi@10.42.10.12
```

`-J` 指定 SSH 跳板，`-tt` 强制分配交互终端。不要把密码写进命令或文档。登录后先核对确实是 MDU：

```bash
hostname
uname -m
ip -br link show taixin_mdu
ip -br addr show taixin_mdu
ip -4 route show dev taixin_mdu
```

只有 HaLow 二层链路修好、电脑已能 ping `.128` 且邻居表已解析出 MAC 后，才可改用直连：

```bash
ssh -tt agi@192.168.100.128
```

当前 `.128` 邻居状态为 `INCOMPLETE`，所以这条直连命令预期失败，不能用它作为本轮维护入口。

```bash
ip -br link show taixin_mdu
ip -br addr show taixin_mdu
ip -4 route show dev taixin_mdu
```

如果接口不存在、`DOWN`、没有 `.128/24`，或驱动参数没有匹配 AP，则机器人尚未加入 A3Test。只有
`taixin_mdu` 为 `UP`、地址为 `.128/24`，且从机器人能 ping `.112` 后，才回到电脑验证 `.128`。
不要用普通 Wi-Fi 接口的 Conference Hall 地址替代 `.128`，也不要为了排障先断开仍可用的维护链路。

现场 MDU 回执已确认：hostname=`mdu`、架构=`aarch64`，`taixin_mdu` 为 `UP,LOWER_UP`，MAC
`ea:5c:43:82:83:c0`，但 IPv4 仍是 `10.42.2.50/24`，直连路由为 `10.42.2.0/24`。因此 `.128`
ping 失败的直接原因已经明确：机器人 HaLow 接口尚未应用设计的 `192.168.100.128/24`。群控页面的
值不能替代接口实际状态。

在修改前，先于 MDU 保存以下只读输出，用于确定 NetworkManager profile 和无线关联：

```bash
nmcli -g GENERAL.CONNECTION,GENERAL.STATE,IP4.ADDRESS,IP4.GATEWAY \
  device show taixin_mdu
nmcli -t -f NAME,UUID,TYPE,DEVICE connection show
iw dev taixin_mdu link || true
sudo hgpriv taixin_mdu
```

如果 `iw` 或无参数 `hgpriv` 不受支持，保留其原始错误输出，不要尝试未知的写命令。只有确认
`GENERAL.CONNECTION` 对应的精确 profile 后，才修改该 profile 的 IPv4；不得根据接口名猜 profile
名称，也不得修改当前维持 HDU→MDU SSH 的其他接口。

现场只读输出已确认：`GENERAL.CONNECTION=nm-taixin_mdu`、state=`connected`（这里仅指 Linux
以太网型接口 profile 已激活）、IPv4=`10.42.2.50/24`；`iw dev taixin_mdu link` 明确显示
`Not connected`，所以 HaLow STA 尚未关联 AP。维持 SSH 的另一 profile 是 `nm-eth_mdu`，禁止修改。
`hgpriv` 帮助确认驱动支持 `set/get ssid,bss_bw,chan_list,mode`，以及只读
`conn_state,center_freq,signal,link_quality` 等参数。

下一步逐条读取机器人无线配置，不查询会泄露凭据的 `wpa_psk`：

```bash
sudo hgpriv taixin_mdu get mode
sudo hgpriv taixin_mdu get ssid
sudo hgpriv taixin_mdu get bss_bw
sudo hgpriv taixin_mdu get chan_list
sudo hgpriv taixin_mdu get key_mgmt
sudo hgpriv taixin_mdu get conn_state
sudo hgpriv taixin_mdu get center_freq
sudo hgpriv taixin_mdu get signal
```

根据这些回执再确定最小无线设置差异；不能在不知道当前 `key_mgmt` 的情况下重写密码。IPv4 修改将
只命中 `nm-taixin_mdu`，并设置 `ipv4.never-default=yes`，避免一个尚不可达的 `.10` 网关抢走 MDU
当前维护默认路由。群控 UI 可以保留设计字段 `.10`，但同一 `/24` 内 `.112↔.128` 不需要网关。

现场查询确认了确切错配：机器人是 `mode=sta`、SSID=`RAGA30P16C6200018`、`BSS_BW=4`、
`CHAN_LIST=9060,9100,9140,9180,9220,9260`、`key_mgmt=WPA-PSK`、center=`9060`；AP 是
`A3Test`、8 MHz、`9080,9160,9240`、channel 1/908 MHz。SSID、带宽和列表全部不一致，故关联必然
失败；`mode` 与 `key_mgmt` 无需改变语义。

#### MDU 最小修复命令

先在电脑 netat 终端重新执行 `AT+PSK=?`，确认 **SSID 已为 `A3Test` 之后**的当前 32 字节 PSK。
现场回执已确认它与当前明文密码经 WPA-PSK 标准 PBKDF2-HMAC-SHA1（SSID=`A3Test`、4096 轮、32 字节）
派生的 64-hex 值一致；敏感值不写入仓库。该值已经被粘贴到聊天，应视为已泄露，链路调通后必须协调
AP 与机器人统一轮换。

保持 HDU→MDU SSH，在 MDU 输入：

```bash
sudo nmcli device disconnect taixin_mdu
sudo ip link set dev taixin_mdu down

sudo hgpriv taixin_mdu set chan_list=9080,9160,9240
sudo hgpriv taixin_mdu set bss_bw=8
sudo hgpriv taixin_mdu set key_mgmt=WPA-PSK

read -rsp 'A3Test password: ' A3TEST_PASSWORD; echo
A3TEST_PSK="$(
  printf '%s' "${A3TEST_PASSWORD}" |
    python3 -c 'import hashlib,sys; print(hashlib.pbkdf2_hmac("sha1", sys.stdin.buffer.read(), b"A3Test", 4096, 32).hex())'
)"
unset A3TEST_PASSWORD
if [[ "${A3TEST_PSK}" =~ ^[0-9A-Fa-f]{64}$ ]]; then
  sudo hgpriv taixin_mdu set wpa_psk="${A3TEST_PSK}"
else
  echo 'ERROR: PSK must be exactly 64 hexadecimal characters' >&2
fi
unset A3TEST_PSK

sudo hgpriv taixin_mdu set ssid=A3Test
sudo hgpriv taixin_mdu set mode=sta

sudo ip link set dev taixin_mdu up
sudo nmcli connection up nm-taixin_mdu
```

`read -s` 隐藏输入，Python 在本机按 WPA 规则派生 PSK，明文密码和 PSK 都不出现在 shell 历史；执行时
仍要避免录屏、shell 调试模式和不可信进程监控。如果 MDU 没有 `python3`，停止并报告，不要退回把
凭据写在命令行。该顺序遵循泰芯 V3.9 驱动指南：连接参数必须在 netdev down 时设置，PSK 在 SSID
之前、mode 最后。随后只读验证：

```bash
sudo hgpriv taixin_mdu get mode
sudo hgpriv taixin_mdu get ssid
sudo hgpriv taixin_mdu get bss_bw
sudo hgpriv taixin_mdu get chan_list
sudo hgpriv taixin_mdu get conn_state
sudo hgpriv taixin_mdu get center_freq
sudo hgpriv taixin_mdu get signal
sudo hgpriv taixin_mdu get sta_list
```

只有 `conn_state=9`（`WPA_COMPLETED`）且 STA 侧 `sta_list` 显示目标 AP 后，才修改 IPv4。精确
profile 已绑定为 `nm-taixin_mdu`。群控 UI 保留设计网关字段 `.10`；MDU NetworkManager profile
使用空 gateway + `never-default`，避免不存在/不可达的 `.10` 抢当前维护路由：

```bash
sudo nmcli connection modify nm-taixin_mdu \
  ipv4.method manual \
  ipv4.addresses 192.168.100.128/24 \
  ipv4.gateway "" \
  ipv4.never-default yes \
  ipv4.dns ""

sudo nmcli connection up nm-taixin_mdu
```

不要修改 `nm-eth_mdu`。应用后在 MDU 验证反向链路：

```bash
ip -br addr show taixin_mdu
ip -4 route show dev taixin_mdu
ping -c 4 192.168.100.112
```

然后在电脑验证：

```bash
ping -c 4 192.168.100.128
ip neigh show 192.168.100.128 dev enp129s0
```

若无线参数设置后仍未关联，停止在 IPv4 修改之前并保留所有回显；不要通过反复重写密码、关闭加密或
断开 Conference Hall 来碰运气。

现场已按上述顺序提交 BSS、信道列表、SSID、`key_mgmt`、隐藏输入的 `wpa_psk` 与 STA mode；报告六次
`RESP:0`。参数随后回读匹配；关联判据按下面 SDK V2 规则解释，不使用标准 `iw link`。

### SDK V2 关联判据纠正（当前权威）

泰芯固件源码定义 `WPA_COMPLETED=9`，并在 STA 有连接时让 `get conn_state` 返回 9；现场当前值正是 9。
泰芯 Linux FMAC 驱动指南 V3.9 同时注明 `get bssid` 仅 SDK V1.x 支持，而 `get sta_list` 在 STA 端用于
查看当前连接的 AP。因此本机 SDK V2 的 BSSID 全零、`iw Not connected`、bitrate=-1 或 link-quality
全零不能否决连接。此前据这些值作出的“尚未关联”结论已撤销，也不要再执行 reset、hwscan、
start_assoc 或 AP→STA toggle。

当前只执行以下只读闭合：

```bash
sudo hgpriv taixin_mdu get conn_state
sudo hgpriv taixin_mdu get sta_list
sudo hgpriv taixin_mdu get center_freq
sudo hgpriv taixin_mdu get signal
```

若 `conn_state=9` 且 `sta_list` 显示目标 AP `ea:5c:43:76:5a:c0`，无线关联通过，直接进入上面的
`nm-taixin_mdu` 静态 IPv4 步骤。若 `sta_list` 不支持或为空，再到 AP 端查询 STA list；不得回退使用
SDK V1-only BSSID 判据。

现场闭合已通过：`conn_state=9`，STA list 精确显示 AP `ea:5c:43:76:5a:c0`，RSSI=`-34 dBm`、
EVM=`-20`、TX/RX SNR=`36/51`；`get signal=-35`。当前中心频点为 9240（924 MHz），仍在双方共同的
`9080,9160,9240` 列表内。无线门已通过，现在允许执行上面的 `nm-taixin_mdu` 静态 IPv4 命令；下一门
是 MDU→电脑 `.112` 反向 ping 与电脑→机器人 `.128` 正向 ping。

按 exact profile `nm-taixin_mdu` 的静态 IPv4 命令应用后，电脑端最终验证：从
`192.168.100.112/enp129s0` 向 `192.168.100.128` 发送 4 个 ICMP 包，收到 4 个、0% 丢包，平均延迟
5.544 ms；邻居表把 `.128` 解析为机器人 `taixin_mdu` MAC `ea:5c:43:82:83:c0`，状态 REACHABLE。
至此 AP 配置、STA 认证、HaLow 二层桥接、机器人运行态静态 IP 和电脑侧路由全部通过。再用下面的
只读命令确认 NetworkManager 持久化字段，之后可直接使用：

```bash
nmcli -g connection.id,ipv4.method,ipv4.addresses,ipv4.gateway,ipv4.never-default \
  connection show nm-taixin_mdu
```

现场持久化回执为 `nm-taixin_mdu` / `manual` / `192.168.100.128/24` / gateway 空 / `yes`，符合安全
设计。gateway 空不是缺陷：`.128↔.112` 同属 `/24`，直接使用二层邻居，不经过 `.10`；群控 UI 的
`.10` 字段与 MDU 默认路由是两个不同层面的配置。MDU 反向 ping `.112` 也取得 3/3、0% loss、平均
5.082 ms。NetworkManager 持久化和双向 IPv4 门均已通过。

```bash
ssh -tt agi@192.168.100.128
```

保留原 HDU→MDU 维护入口作为回退；不要删除或修改 `nm-eth_mdu`。当前密码曾出现在聊天记录中，现场
业务验证完成后应协调 AP 和所有 STA 同步轮换，而不是只改一端。

<details>
<summary>现场早期误判与排查历史（已撤销；不要执行下面的命令）</summary>

以下记录保留用于解释为何做过 reset/scan/mode toggle，不再作为操作步骤。

参数回读随后确认写入已生效：`mode=sta`、SSID=`A3Test`、`BSS_BW=8`、
`CHAN_LIST=9080,9160,9240`、`center_freq=9080`、signal=`-35 dBm`；`conn_state=9`，但
`iw` 仍显示 `Not connected`。该接口以 Ethernet profile 暴露且连接状态枚举未有本地说明，因此不能
用 `iw` 单独否决。继续读取驱动原生关联证据：

```bash
sudo hgpriv taixin_mdu get bssid
sudo hgpriv taixin_mdu get stainfo
sudo hgpriv taixin_mdu get link_quality
sudo hgpriv taixin_mdu get tx_bitrate
sudo hgpriv taixin_mdu get disassoc_reason
```

若 `bssid` 等于 AP MAC `ea:5c:43:76:5a:c0`，并有合理的 stainfo/link quality，则把它作为关联通过
依据；若 BSSID 全零/空或存在断联原因，停止并保留回显。仍不查询 `wpa_psk`。

现场原生回执进一步确认尚未关联：BSSID=`00:00:00:00:00:00`、`tx_bitrate=-1`，link quality 中
RSSI/EVM/PER/BSS station 数全部为 0；`stainfo` 返回 `-22`，`disassoc_reason=0`。因此先不要修改
`nm-taixin_mdu` 的 IPv4，也不能把 `signal=-35` 或 `center_freq=9080` 当成已关联证据。下一门先读取
模块/固件/频率范围、缓存扫描结果及状态码：

```bash
sudo hgpriv taixin_mdu get module_type
sudo hgpriv taixin_mdu get fwinfo
sudo hgpriv taixin_mdu get freq_range
sudo hgpriv taixin_mdu get scan_list
sudo hgpriv taixin_mdu get status_code
sudo hgpriv taixin_mdu get reason_code
```

泰芯官方 [《AH 方案 FAQ》](https://taixin-semi.com/zh/docs/txw8301/latest/%E6%B3%B0%E8%8A%AFAH%E6%96%B9%E6%A1%88FAQ)
说明：STA 正处于连接过渡状态时修改 SSID 或 key/PSK，可能使连接状态卡死；规避顺序是先将 HaLow
接口置 down，写完参数后再置 up。现场第一次写六项参数时 `taixin_mdu` 一直为 UP，和该已知问题吻合。
因此下一次重写必须保留 `nm-eth_mdu` 维护 SSH，只短暂断开 `taixin_mdu`：

```bash
sudo -v
(
  set -e
  restore_taixin() {
    sudo ip link set dev taixin_mdu up || true
    sudo nmcli connection up nm-taixin_mdu || true
  }
  trap restore_taixin EXIT

  sudo nmcli device disconnect taixin_mdu
  sudo ip link set dev taixin_mdu down

  sudo hgpriv taixin_mdu set bss_bw=8
  sudo hgpriv taixin_mdu set chan_list=9080,9160,9240
  sudo hgpriv taixin_mdu set ssid=A3Test
  sudo hgpriv taixin_mdu set key_mgmt=WPA-PSK

  read -rsp 'A3Test password: ' A3TEST_PASSWORD; echo
  A3TEST_PSK="$(
    printf '%s' "${A3TEST_PASSWORD}" |
      python3 -c 'import hashlib,sys; print(hashlib.pbkdf2_hmac("sha1", sys.stdin.buffer.read(), b"A3Test", 4096, 32).hex())'
  )"
  unset A3TEST_PASSWORD
  [[ "${A3TEST_PSK}" =~ ^[0-9A-Fa-f]{64}$ ]]
  sudo hgpriv taixin_mdu set wpa_psk="${A3TEST_PSK}"
  unset A3TEST_PSK

  sudo ip link set dev taixin_mdu up
  sudo nmcli connection up nm-taixin_mdu
  trap - EXIT
)

sleep 8
sudo hgpriv taixin_mdu get bssid
sudo hgpriv taixin_mdu get conn_state
sudo hgpriv taixin_mdu get link_quality
sudo hgpriv taixin_mdu get tx_bitrate
```

该块不修改任何 IPv4 配置；异常退出时 trap（退出清理）会尝试恢复 `taixin_mdu` 和它的精确 profile。
如果执行后 BSSID 仍全零，官方 FAQ 指向“模块需要重启”而不是继续改 IP。此时先停止并保留回显；在
确认本机驱动的模块重启命令及影响前，不猜 `reset` 参数、不重启整台机器人。若恢复后能关联但仍无 IP，
才继续本手册前面的 `nm-taixin_mdu` IPv4 步骤。

现场随后按上述 down→重写→up 完成，五项写入均为 `RESP:0`，但八秒后仍是 BSSID 全零、
`conn_state=9`、link quality 全零、bitrate=`-1`，因此规避流程没有清除已经卡住的模块状态。泰芯官方
[`TXW8301_FMAC@63b0ef5`](https://github.com/Taixin-Semiconductor/TXW8301_FMAC/blob/63b0ef5b948c2ffdfaf47bfe86ea937f99e4cdc2/sdk/include/lib/lmac/hgic.h#L128)
命令枚举确认 `LOAD_DEF`（恢复默认）、`SAVE_CFG`（保存配置）和 `RESET`（重启模块）是三个独立命令；
现场 `hgpriv` 帮助也已枚举 `reset`。下一门只重启 HaLow 模块，不执行 `loaddef`，先观察现有参数是否
由模块保存：

```bash
sudo -v
(
  set -e
  restore_taixin() {
    sudo ip link set dev taixin_mdu up || true
    sudo nmcli connection up nm-taixin_mdu || true
  }
  trap restore_taixin EXIT

  sudo nmcli device disconnect taixin_mdu
  sudo ip link set dev taixin_mdu down
  sudo hgpriv taixin_mdu set reset=1
  sleep 10
  sudo ip link set dev taixin_mdu up
  sudo nmcli connection up nm-taixin_mdu
  trap - EXIT
)

sleep 8
sudo hgpriv taixin_mdu get mode
sudo hgpriv taixin_mdu get ssid
sudo hgpriv taixin_mdu get bss_bw
sudo hgpriv taixin_mdu get chan_list
sudo hgpriv taixin_mdu get bssid
sudo hgpriv taixin_mdu get conn_state
sudo hgpriv taixin_mdu get link_quality
sudo hgpriv taixin_mdu get tx_bitrate
```

`reset=1` 必须先返回 `RESP:0`。重启后若 SSID/BSS/列表仍正确且 BSSID 匹配 AP，关联门通过；若参数
回退，保持 IPv4 不变并重新执行一次 down→重写→up；若参数正确但 BSSID 仍全零，则停止在扫描/显式
关联诊断，不重复 reset。

现场模块 reset 返回 `RESP:0`，且 `conn_state` 从 9 变为 0，说明卡住状态已清除。SSID `A3Test` 被
保留，但 BSS 回退为 4 MHz、列表回退为 `9060,9100,9140,9180,9220,9260`；BSSID 仍全零是当前
参数再次错配的预期结果，不是 reset 失败。此时不得再次 reset；在模块已清醒状态下重新执行上面的
down→写入 8 MHz/三频点/SSID/key-mgmt/PSK→up 块，然后重新检查 BSSID、连接状态、链路质量和速率。

复写后 BSS=8、列表=`9080,9160,9240` 已再次匹配 AP，但 BSSID 仍全零、`conn_state=9`、link
quality 全零、bitrate=`-1`。因此下一门不是 IP 或再次重写凭据，而是用驱动已公开枚举的 `hwscan`
做一次主动扫描，确认射频层是否能看到目标 AP：

```bash
sudo hgpriv taixin_mdu get scan_list
sudo hgpriv taixin_mdu get status_code
sudo hgpriv taixin_mdu get reason_code

sudo hgpriv taixin_mdu set hwscan=1
sleep 5

sudo hgpriv taixin_mdu get scan_list
sudo hgpriv taixin_mdu get status_code
sudo hgpriv taixin_mdu get reason_code
```

`hwscan=1` 是扫描，不改 IPv4、不恢复出厂。若返回 `RESP:0` 且结果出现 `A3Test`/9080 MHz，再验证
驱动枚举的 `start_assoc` 显式关联；若扫描结果没有目标 AP，则停止在 AP 广播/射频可见性排查，不能
用 `start_assoc`、重复 reset 或继续改密码碰运气。

现场 `scan_list` 已明确列出 `A3Test`、BSSID `ea:5c:43:76:5a:c0`、9080 MHz、signal `-35 dBm`；
同时还看到另外两个 9080 MHz AP。`status_code=0`、`reason_code=0`。主动 `hwscan=1` 返回 `-524`，但
前后缓存扫描表均完整且目标稳定可见，所以射频、AP 广播、频率和信号门均已通过；不需要重复 hwscan。
下一步只显式发起一次关联并读取结果：

```bash
sudo hgpriv taixin_mdu set start_assoc=1
sleep 8

sudo hgpriv taixin_mdu get bssid
sudo hgpriv taixin_mdu get conn_state
sudo hgpriv taixin_mdu get link_quality
sudo hgpriv taixin_mdu get tx_bitrate
sudo hgpriv taixin_mdu get status_code
sudo hgpriv taixin_mdu get reason_code
sudo hgpriv taixin_mdu get disassoc_reason
```

若 `start_assoc=1` 返回 `RESP:0`，BSSID 应变为目标 AP MAC 且 bitrate 应大于 0。若仍为零 BSSID，保留
所有状态/原因码并停止；此时射频已排除，剩余主要是认证/PSK 或关联状态机，不再改频段、IP 或 reset。

现场 `start_assoc=1` 返回 `-524`，和 `hwscan` 相同，表示当前 Linux 驱动/固件组合不支持该可选命令；
状态仍是零 BSSID、`conn_state=9`、零 link quality、bitrate=`-1`，所有 reason/status 均为 0。泰芯官方
[`wificfg_set_mode`](https://github.com/Taixin-Semiconductor/TXW8301_FMAC/blob/63b0ef5b948c2ffdfaf47bfe86ea937f99e4cdc2/project/syscfg.c#L383-L416)
实现显示：写入与当前相同的 mode 会直接返回，只有 mode 真正变化才 stop/start 固件无线接口。此前 reset
后 mode 已是 STA，因此复写末尾没有再次启动 STA 状态机。保持 Linux netdev down，短暂执行 AP→STA
以 STA 作为最后一项并真正重启关联状态机：

```bash
sudo -v
(
  set -e
  restore_taixin() {
    sudo hgpriv taixin_mdu set mode=sta || true
    sudo ip link set dev taixin_mdu up || true
    sudo nmcli connection up nm-taixin_mdu || true
  }
  trap restore_taixin EXIT

  sudo nmcli device disconnect taixin_mdu
  sudo ip link set dev taixin_mdu down
  sudo hgpriv taixin_mdu set mode=ap
  sleep 1
  sudo hgpriv taixin_mdu set mode=sta
  sleep 2
  sudo ip link set dev taixin_mdu up
  sudo nmcli connection up nm-taixin_mdu
  trap - EXIT
)

sleep 10
sudo hgpriv taixin_mdu get mode
sudo hgpriv taixin_mdu get bss_bw
sudo hgpriv taixin_mdu get chan_list
sudo hgpriv taixin_mdu get bssid
sudo hgpriv taixin_mdu get conn_state
sudo hgpriv taixin_mdu get link_quality
sudo hgpriv taixin_mdu get tx_bitrate
sudo hgpriv taixin_mdu get status_code
sudo hgpriv taixin_mdu get reason_code
sudo hgpriv taixin_mdu get disassoc_reason
```

AP mode 只在 Linux `taixin_mdu` down 时短暂存在一秒，trap 会在任何失败路径先恢复 STA 再恢复 exact
profile；不修改维护接口或 IPv4。该门只执行一次，不重复 mode toggle。

</details>

## 9. 本次验证边界

2026-08-24 已完成：三张截图人工转录、当前源码定位、本机网口只读检查、源码编译，以及 AP 发现。
第一次仍使用不存在的旧接口启动，得到 `SO_BINDTODEVICE error:[19:No such device]`；把最后一次赋值改为
`enp129s0` 后再次编译运行，得到 `init success`、发现一个设备并自动选择 device 1，MAC
`ea:5c:43:76:5a:c0`。源码仍保留前一条 `enp0s31f6` 赋值，所以有字符串常量到 `char *` 的两个编译
警告；后一次赋值覆盖前一次，未阻止本轮发现。随后取得只读 AT 基线：固件
`v2.4.1.3-41928`，AP 模式、SSID `SJTU-RL2-LAB`、8 MHz、三频点、加密开启，且 `PSK` / `KEY`
查询均受支持；敏感值不落文档。没有发送 AT 设置、SSH 登录 MDU、修改遥控器配置或运行机器人命令。
编译运行的现场原命令为
`g++ linux_netat.cpp -o linux_netat.o && ./linux_netat.o`。

泰芯公开参考：
[《泰芯 AH-SDK V2.x AT 指令使用说明 V1.3》](https://taixin-semi.com/zh/docs/txw8301/latest/%E6%B3%B0%E8%8A%AFah-sdk-v2-x-at%E6%8C%87%E4%BB%A4%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E-v1-3)。
