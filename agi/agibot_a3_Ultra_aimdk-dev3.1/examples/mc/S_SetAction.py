#!/usr/bin/env python3

## 功能：设置action

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

def get_available_actions():
    url = f"http://10.42.10.12:56322/rpc/aimdk.protocol.MotionControlActionService/GetAvailableActions"
    headers = {'Content-Type': 'application/json'}
    response = requests.post(url, headers=headers, json={})
    response.raise_for_status()

    return response.json().get('commands', [])

def select_action(actions = None):
    try:
        if actions is None:
            actions = get_available_actions()

        if len(actions) == 0:
            print("No actions available.")
            exit(0)
        elif len(actions) == 1:
            return actions[0]["ext_action"]
        else:
            print("Please select an action:")
            for index, action in enumerate(actions, 1):
                print(f" {index:02d}: {action['ext_action']}")
            choice = input("Enter the number corresponding to the desired action: ")
            if choice.isdigit() and int(choice) >= 1 and int(choice) <= len(actions):
                return set_action(actions[int(choice) - 1])
            elif isinstance(choice, str) and choice == "q":
                exit(0)
            else:
                print("Invalid choice.")
                exit(1)
    except Exception as e:
        print(f"Error: {e}")
        exit(1)

def set_action(action):
    url = f"http://10.42.10.12:56322/rpc/aimdk.protocol.MotionControlActionService/SetAction"
    headers = {"Content-Type": "application/json"}
    payload = {
        "header": create_header(),
        "command": action,
    }
    response = requests.Session().post(url, headers=headers, json=payload)
    return response.json()


def pretty_print_json(json_data):
    print("Response:")
    print(json.dumps(json_data, indent=2, ensure_ascii=False))


def main():
    response = select_action()
    pretty_print_json(response)
    requests.Session().close()

if __name__ == "__main__":
    main()
