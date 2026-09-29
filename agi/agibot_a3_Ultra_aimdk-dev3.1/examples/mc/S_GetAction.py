#!/usr/bin/env python3

## 功能：查询当前action状态

import json
import requests
from datetime import datetime

def create_header():
    now = datetime.utcnow()
    header = {
        "timestamp": {
            "seconds": int(now.timestamp()),
            "nanos": now.microsecond * 1000,
            "ms_since_epoch": int(now.timestamp() * 1000),
        },
        "control_source": "ControlSource_SAFE",
        "uuid": "",
        "trace_id": "user_McScript",
        "domin": "",
    }
    return header

def service_call():
    url = 'http://10.42.10.12:56322/rpc/aimdk.protocol.MotionControlActionService/GetAction'
    headers = {'Content-Type': 'application/json'}
    payload = {
        "header": create_header(),
    }
    
    response = requests.Session().post(url, headers=headers, json=payload)
    response.raise_for_status()
    
    return response.json()

def pretty_print_json(json_data):
    print("Response:")
    print(json.dumps(json_data, indent=2, ensure_ascii=False))


def main():
    response = service_call()
    pretty_print_json(response)
    requests.Session().close()

if __name__ == "__main__":
    main()
