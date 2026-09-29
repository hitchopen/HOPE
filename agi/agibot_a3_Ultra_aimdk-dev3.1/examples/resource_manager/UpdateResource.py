#!/usr/bin/env python3

import requests
import json
import time

RPC_URL = "http://10.42.10.10:51049/rpc/aimdk.protocol.ResourceService/UpdateResource"

HEADERS = {
    "Content-Type": "application/json"
}

def update_resource():
    payload = {
        "header": {
            "timestamp": {
                "seconds": "0",
                "nanos": 0,
                "ms_since_epoch": str(int(time.time() * 1000))
            },
            "control_source": 0
        },
        "resource": {
            "resource_id": 10001,
            "resource_type": "RESOURCE_TYPE_EMOTICON",
            "resource_name": "2222222222222222",
            "emoticon_extra_info": {
                "emoticon_file_url": "",
                "thumbnail_file_url": "",
                "cover_file_url": ""
            }
        }
    }

    print("Sending request:")
    print(json.dumps(payload, indent=2, ensure_ascii=False))

    resp = requests.post(RPC_URL, headers=HEADERS, json=payload, timeout=10)

    print("Response result:")
    try:
        print(json.dumps(resp.json(), indent=2, ensure_ascii=False))
    except Exception:
        print(resp.text)


if __name__ == "__main__":
    update_resource()
