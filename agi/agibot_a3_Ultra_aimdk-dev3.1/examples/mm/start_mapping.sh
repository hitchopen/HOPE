#!/bin/bash

curl --location --request POST 'http://10.42.10.11:50807/rpc/aimdk.protocol.MappingService/StartMapping' \
  --header 'Content-Type: application/json' \
  --data-raw '{
    "header": {
      "timestamp": {
        "seconds": "0",
        "nanos": 0,
        "ms_since_epoch": "1744598548952"
      }
    },
    "command": "MappingCommand_START_MAPPING",
    "no_realtime_data": true
  }'
