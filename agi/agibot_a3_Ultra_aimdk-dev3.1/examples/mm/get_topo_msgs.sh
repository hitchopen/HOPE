#!/bin/bash

if [ $# -eq 0 ]; then
    echo "arg error, need map_id, ./get_topo_msgs.sh map_id"
    exit 0
fi

# 获取当前地图的拓扑数据
curl --location --request POST 'http://10.42.10.11:50807/rpc/aimdk.protocol.LocalizationService/GetTopoMsgs' \
     --header 'Content-Type: application/json' \
     --data-raw '{"header":{},"command":"TopoCommand_GET_TOPO_MSG","map_id":'$1'}'
