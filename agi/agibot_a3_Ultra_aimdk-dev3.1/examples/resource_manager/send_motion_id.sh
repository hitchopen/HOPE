#!/bin/bash

if [ $# -eq 0 ]; then
    echo "arg error, need motion_id, ./send_motion_id.sh motion_id, example: "
    echo ./send_motion_id.sh /agibot/data/resources/default/motion/演讲10s/演讲10s.mcap
    exit 0
fi

MC_ID="$1"

if [ "$MC_ID" == "停止动作" ]; then
    # 如果是“停止动作”，则 motion_id 为空，cmd_reset 设为 true
    DATA='{
      "motion_id": "",
      "duration_ms": 10000,
      "cmd_end": true,
      "cmd_pause": false,
      "cmd_reset": true
    }'
elif [ "$MC_ID" == "暂停播放器" ]; then
    # 如果是“暂停播放器”，则 motion_id 保持不变，cmd_pause 设为 true
    DATA='{
      "motion_id": "",
      "duration_ms": 10000,
      "cmd_end": true,
      "cmd_pause": true,
      "cmd_reset": false
    }'
elif [ "$MC_ID" == "下一个动作" ]; then
    # 如果是“下一个动作”，则 播放list中的下一个动作
    DATA='{
      "motion_id": "next_motion",
      "duration_ms": 10000,
      "cmd_end": true,
      "cmd_pause": false,
      "cmd_reset": false
    }'
else
    # 如果不是“停止动作”或“暂停播放器”，保持原有逻辑
    DATA='{
      "motion_id": "'"$MC_ID"'",
      "duration_ms": 10000,
      "cmd_end": true,
      "cmd_pause": false,
      "cmd_reset": false
    }'
fi

# 使用 curl 发送 POST 请求
curl -i \
    -H 'content-type:application/json' \
    -H 'timeout: 1000' \
    -X POST http://10.42.10.12:56444/rpc/aimdk.protocol.MotionCommandService/SendMotionCommand \
    -d "$DATA"

# 打印发送的数据
echo "$DATA"
