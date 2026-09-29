#!/bin/bash

curl -X POST 'http://10.42.10.11:50807/rpc/aimdk.protocol.MappingService/StopMapping' \
  -H 'Content-Type: application/json' \
  -d '{
    "header": {
      "timestamp": {
        "seconds": "0",
        "nanos": 0,
        "ms_since_epoch": "1744598548952"
      }
    },
    "command": "MappingCommand_SAVING_MAP",
    "map_name": "测试地图"
  }'