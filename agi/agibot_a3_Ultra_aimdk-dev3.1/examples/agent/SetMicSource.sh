#!/bin/bash

# 0 for internal mic, 1 for external mic
curl --location --request POST 'http://10.42.10.10:59301/rpc/aimdk.protocol.AgentControlService/SetMicSourceRequest' \
     --header 'Content-Type: application/json' \
     --data-raw '{"mic_source": 1}'
