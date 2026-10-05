#!/bin/bash

if [ $# -eq 0 ]; then
    echo "arg error, need file_name, ./play_media_file.sh file_name"
    exit 0
fi

curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://10.42.10.10:59301/rpc/aimdk.protocol.TTSService/PlayMediaFile' \
    -d '{"file_name": "'$1'","priority_level":"INTERACTION_L6","domain":"example", "trace_id":"hafhjkqwjwefk", "is_interrupted": true}'
