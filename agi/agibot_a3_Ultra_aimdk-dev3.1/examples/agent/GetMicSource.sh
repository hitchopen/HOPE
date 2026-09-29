#!/bin/bash

curl --location --request POST 'http://10.42.10.10:59301/rpc/aimdk.protocol.AgentControlService/GetMicSourceRequest' \
     --header 'Content-Type: application/json' \
     --data-raw '{}'
