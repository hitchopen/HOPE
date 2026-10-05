curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://10.42.10.10:56666/rpc/aimdk.protocol.HalAudioService/PlayFile' \
    -d '{
    "header": {
        "timestamp": {
            "seconds": "0",
            "nanos": 0,
            "ms_since_epoch": "0"
        }
    },
    "pkg_name": "",
    "file_name": "Ding.wav",
    "file_path": "",
    "priority": "DEFAULT",
    "priority_weight": 0,
    "channles": 0,
    "samplerate": 0
}'

# 当前pcm播放存在bug，pcm播放请使用7.2.3音频文件播放接口
