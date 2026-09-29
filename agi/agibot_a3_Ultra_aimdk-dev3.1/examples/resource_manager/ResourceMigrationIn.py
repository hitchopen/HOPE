#!/usr/bin/env python3

import requests
import json
import time

RPC_URL = "http://10.42.10.10:51049/rpc/aimdk.protocol.ResourceService/ResourceMigrationIn"

HEADERS = {
    "Content-Type": "application/json"
}

def resource_migration_in():
    payload = {
        "header": {
            "timestamp": {
                "seconds": "0",
                "nanos": 0,
                "ms_since_epoch": str(int(time.time() * 1000))
            },
            "control_source": 0
        },
        "migration_file": {
            "file_url": "tmp/1767839589.gz"
        }
    }

    print("Sending request:")
    print(json.dumps(payload, indent=2, ensure_ascii=False))

    resp = requests.post(
        RPC_URL,
        headers=HEADERS,
        json=payload,
        timeout=60
    )

    print("Response result:")
    try:
        print(json.dumps(resp.json(), indent=2, ensure_ascii=False))
    except Exception:
        print(resp.text)


if __name__ == "__main__":
    resource_migration_in()
