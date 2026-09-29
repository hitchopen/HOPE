#!/usr/bin/env python3

import requests
import json
import time
from enum import Enum

RPC_URL = "http://10.42.10.10:51049/rpc/aimdk.protocol.ResourceService/CreateResource"

HEADERS = {
    "Content-Type": "application/json"
}
#the resources used for the following actions, emoticons, and audio are provided in the folders under examples/resource_manager/resource.
#you can also find the corresponding resource folders under the robot path agito/data/resources/default/.
#the resource_name in the code has been replaced for testing purposes.
RESOURCE_LIBRARY = {

    "emotion": {
        "resource_name": "测试",
        "resource_type": "RESOURCE_TYPE_EMOTICON",
        "tags":["测试", "表情"],
        "emoticon_extra_info":{
            "emoticon_file_url":"tmp/emoticon.mp4",
            "thumbnail_file_url":"tmp/thumbnail.mp4",
            "cover_file_url":"tmp/cover.png"
        }
    },

    "motion": {
        "resource_type": "RESOURCE_TYPE_MOTION",
        "resource_name": "动作测试",
        "motion_extra_info":{
           "record_file_path":"/agibot/data/home/agi/动作测试/右手点赞_长",
        }
    },

    "audio": {
        "resource_type": "RESOURCE_TYPE_AUDIO",
        "resource_name": "音频测试",
        "audio_extra_info": {
            "audio_file_url":"tmp/趣味问候01.wav"             
            }   
    },

    "motion_work": {
        "resource_name": "测试创作作品",
        "resource_type": "RESOURCE_TYPE_OFFRING_WORK",
        "description": "用于测试的创作作品资源",
        "offring_work_extra_info": {
            "cover_file_url": "",
            "preview_video_url": "",
            "display_name": {
                "display_name_zh": "测试创作作品",
                "display_name_en": "Test Creative Work"
            },
            "motions": [
                {
                    "type": "NormalMotion",
                    "resources": [
                        {
                            "source_type": 1,
                            "path": "/agibot/data/resources/default/motion/右手碰拳/右手碰拳.mcap"
                        }
                    ]
                }
            ],
            "emoticons": [
                {
                    
                    "resources": [
                        {
                            "source_type": 1,
                            "path": "/agibot/data/resources/default/emoticon/emoticon_guiding/emoticon.mp4"
                        }
                    ]
                }
            ],
            "audios": [
                {
                    "resources": [
                        {
                            "source_type": 1,
                            "path": "/agibot/data/resources/default/audio/趣味问候01/趣味问候01.wav"
                        }
                    ]
                }
            ],
        }
    },

    "dance_work": {
        "resource_name": "测试创作作品",
        "resource_type": "RESOURCE_TYPE_OFFRING_WORK",
        "description": "用于测试的创作作品资源",
        "offring_work_extra_info": {
            "cover_file_url": "",
            "preview_video_url": "",
            "display_name": {
                "display_name_zh": "测试创作作品",
                "display_name_en": "Test Creative Work"
            },
            "motions": [
                {
                    "type": "WholeBodyDance",
                    "resources": [
                        {
                            "source_type": 1,
                            "path": "加速时刻"
                        }
                    ]
                }
            ],
            "emoticons": [
                {
                    
                    "resources": [
                        {
                            "source_type": 1,
                            "path": "/agibot/data/resources/default/emoticon/emoticon_guiding/emoticon.mp4"
                        }
                    ]
                }
            ],
            "audios": [
                {
                    "resources": [
                        {
                            "source_type": 1,
                            "path": "/agibot/data/resources/default/audio/趣味问候01/趣味问候01.wav"
                        }
                    ]
                }
            ],
        }
    }   
}

def build_create_payload(resource_key: str) -> dict:
    resource = RESOURCE_LIBRARY[resource_key]

    payload = {
        "header": {
            "timestamp": {
                "seconds": "0",
                "nanos": 0,
                "ms_since_epoch": str(int(time.time() * 1000))
            },
            "control_source": 0
        },
        "resource": resource
    }

    return payload

def create_resource(resource_key: str):
    payload = build_create_payload(resource_key)

    print("Sending request:")
    print(json.dumps(payload, indent=2, ensure_ascii=False))

    resp = requests.post(RPC_URL, headers=HEADERS, json=payload, timeout=5)

    print("Response result:")
    print(json.dumps(resp.json(), indent=2, ensure_ascii=False))


def main():
    create_resource("emotion")


if __name__ == "__main__":
    main()
