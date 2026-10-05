#!/bin/bash

# 需传入指定 trace_id 的播报任务
# ./stop_tts.sh hafhjkqwjwefk

if [ $# -eq 0 ]; then
    echo "arg error, need trace_id, ./stop_tts.sh trace_id, example: "
    echo ./stop_tts.sh hafhjkqwjwefk
    exit 0
fi

curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://10.42.10.10:59301/rpc/aimdk.protocol.TTSService/StopTTSTraceId' \
    -d '{"trace_id":"'$1'"}'
