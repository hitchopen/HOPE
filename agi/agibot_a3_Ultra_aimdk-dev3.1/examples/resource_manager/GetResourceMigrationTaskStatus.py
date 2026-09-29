#!/usr/bin/env python3

import requests
import json
import time

RPC_URL = "http://10.42.10.10:51049/rpc/aimdk.protocol.ResourceService/GetResourceMigrationTaskStatus"

HEADERS = {
    "Content-Type": "application/json"
}

def get_migration_task_status():
    payload = {
        "header": {
            "timestamp": {
                "seconds": "0",
                "nanos": 0,
                "ms_since_epoch": str(int(time.time() * 1000))
            },
            "control_source": 0
        },
        "task_id": "65987072-f9e6-495f-99ed-df1a437fd337"
    }

    print("Sending request:")
    print(json.dumps(payload, indent=2, ensure_ascii=False))

    resp = requests.post(
        RPC_URL,
        headers=HEADERS,
        json=payload,
        timeout=10
    )

    print("Response result:")
    try:
        print(json.dumps(resp.json(), indent=2, ensure_ascii=False))
    except Exception:
        print(resp.text)


if __name__ == "__main__":
    get_migration_task_status()
