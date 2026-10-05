#!/bin/bash

curl --location --request POST 'http://10.42.10.10:56666/rpc/aimdk.protocol.HalAudioService/GetAudioVolume' \
     --header 'Content-Type: application/json' \
     --data-raw '{}'
