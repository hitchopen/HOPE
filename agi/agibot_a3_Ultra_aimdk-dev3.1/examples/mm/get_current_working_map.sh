#!/bin/bash

# 获取当前正在工作的地图id
curl --location --request POST 'http://10.42.10.11:50807/rpc/aimdk.protocol.MappingService/GetCurrentWorkingMap' \
     --header 'Content-Type: application/json' \
     --data-raw '{"header":{},"command":"MappingCommand_GET_CURRENT_WORKING_MAP"}'