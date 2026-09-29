#!/bin/bash

curl -X POST 'http://10.42.10.11:50807/rpc/aimdk.protocol.MappingService/RenameMap' \
  -H 'Content-Type: application/json' \
  -d '{
    "header": {
      "timestamp": {
        "seconds": "0",
        "nanos": 0,
        "ms_since_epoch": "1744598548952"
      }
    },
    "command": "MappingCommand_RENAME_MAP",
    "map_id": "1772000978418", 
    "old_name": "213",
    "new_name": "321",
  }'