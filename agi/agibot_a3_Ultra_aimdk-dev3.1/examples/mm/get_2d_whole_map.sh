#!/bin/bash

if [ $# -eq 0 ]; then
    echo "arg error, need map_id, ./get_2d_whole_map.sh map_id"
    exit 0
fi

# 获取2D地图数据
curl --location --request POST 'http://10.42.10.11:50807/rpc/aimdk.protocol.MappingService/Get2DWholeMap' \
     --header 'Content-Type: application/json' \
     --data-raw '{"header":{},"command":"MappingCommand_GET_2D_WHOLE_MAP","map_id":'$1'}'
