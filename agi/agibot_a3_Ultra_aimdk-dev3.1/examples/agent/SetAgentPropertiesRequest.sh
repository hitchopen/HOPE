#!/bin/bash

# 设置 agent 模块为 only_voice 模式.d=调用后重启机器人生效
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://10.42.10.10:59301/rpc/aimdk.protocol.AgentControlService/SetAgentPropertiesRequest' \
    -d '{ "contents": { "properties": { "2": "only_voice" } } }'
