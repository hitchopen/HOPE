#!/bin/bash

# false for disable voice, true for enable voice
curl --location --request POST 'http://10.42.10.10:59301/rpc/aimdk.protocol.AgentControlService/SetVoiceEnable' \
     --header 'Content-Type: application/json' \
     --data-raw '{"enable_voice": false}'
