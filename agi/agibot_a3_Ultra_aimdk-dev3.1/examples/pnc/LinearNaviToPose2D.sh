#!/bin/bash
#curl使用示例(以 ORIN ip: 192.168.100.110 为例)：
curl -i -H 'content-type:application/json' \
        -H 'timeout: 60000' \
        -X POST 'http://10.42.10.11:53176/rpc/aimdk.protocol.PncService/LinearNaviToPose2D' \
        -d '{
              "header": {
                "timestamp": {
                  "seconds": 0,
                  "nanos": 0,
                  "ms_since_epoch": 0
                },
                "control_source": 0
              },
              "task_id": 0,
              "map_id": 1,
              "pose": {
                "position": {
                  "x": 1.0,
                  "y": 1.0
                },
                "angle": 1.57
              }
            }'