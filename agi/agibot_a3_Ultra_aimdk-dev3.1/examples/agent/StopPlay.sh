curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://10.42.10.10:56666/rpc/aimdk.protocol.HalAudioService/StopPlay' \
    -d '{}'

# 当前pcm播放存在bug，pcm播放请使用7.2.3音频文件播放接口
