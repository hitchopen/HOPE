# 旧 P1 2026-11-10 历史 helper 重建

这是当次最终 staging 的两个 helper 的可重建记录，**不是安装入口**。
`provenance.json` 固定基线 commit、基线/历史文件 hash 和厂商 ABI；
`historical.patch` 精确恢复当时的 `hope-lifecycle` 与 hardened scheduler。
仅在临时离线 checkout 对该固定基线应用补丁，再逐文件验证 historical_sha256。
不要对当前分支或机器人直接应用。

旧 P1 与 20261030 P1 BSP 不是同一 profile，默认 T3 也不能替代它。历史差异包括
P1 型号/配置路径、best-v1 固定 Runner hash 的 CLI 兼容、选中槽位解析、现场参数及
DDS/录制调用。当时的 STOP/解析逻辑后来已有改进，整份倒灌会回退现有行为。
再次部署应基于当前源码单独审查这些差异，生成匹配机器的构建/安装清单并重新验收。

冻结目录还包含旧的 `hope_elink_scheduler_p1_field.py` 和单独安装脚本；最终对齐使用
的是这里归档的 hardened `hope_elink_scheduler.py`。旧安装脚本不作为新的推荐入口，
避免重新引入较弱的 Python/安装边界。该记录不含二进制、密钥、原始日志或全机备份。

`source-disposition.json` 记录冻结目录 216 个文件与本次回收结果的分类：170 个逐字
相同，46 个保留有解释的差异（含当前实现、已补回并加强的构建脚本、硬件例外和旧测试）。
差异不是自动判定的遗漏，也不是已经部署的证明。完整逐文件比较输出随本次任务回执保存。
