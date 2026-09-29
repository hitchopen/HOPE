#! /usr/bin/env python
# -*- coding: utf-8 -*-

import time

import requests
from retry_decorator import retry


@retry(max_attempts=1, delay=1.0)
def set_action(action_name: str):
    url = "http://10.42.10.11:56322/rpc/aimdk.protocol.McActionService/SetAction"
    headers = {"Content-Type": "application/json"}
    payload = {
        "command": {"action": "McAction_USE_EXT_CMD", "ext_action": action_name},
    }
    rsp = requests.post(url, headers=headers, json=payload)
    rsp.raise_for_status()
    return rsp


@retry(max_attempts=1, delay=1.0)
def get_action():
    url = "http://10.42.10.11:56322/rpc/aimdk.protocol.McActionService/GetAction"
    headers = {"Content-Type": "application/json"}
    rsp = requests.post(url, headers=headers, json={})
    rsp.raise_for_status()
    return rsp.json()["info"]["current_action"]


def ensure_action(action_name: str, retries=5, retry_interval=1.0):
    try:
        for i in range(retries):
            if get_action() == action_name:
                return True
            set_action(action_name)
            time.sleep(0.1 if i == 0 else retry_interval)
    except Exception as e:
        print(f"Failed to set action: {e}")
        return False
    return False


def get_current_map_id():
    url = "http://10.42.10.11:50807/rpc/aimdk.protocol.MappingService/GetCurrentWorkingMap"
    headers = {"Content-Type": "application/json"}
    data = {"header": {}, "command": "MappingCommand_GET_CURRENT_WORKING_MAP"}
    rsp = requests.post(url, headers=headers, json=data)
    rsp.raise_for_status()
    return rsp.json()["data"]["map_id"]


def get_navi_points(map_id: int):
    url = "http://10.42.10.11:50807/rpc/aimdk.protocol.LocalizationService/GetTopoMsgs"
    headers = {"Content-Type": "application/json"}
    data = {"header": {}, "command": "TopoCommand_GET_TOPO_MSG", "map_id": map_id}
    rsp = requests.post(url, headers=headers, json=data)
    rsp.raise_for_status()
    return rsp.json()["data"]["points"]


@retry(max_attempts=3, delay=1.0)
def planning_navi_to_goal(map_id: int, target_id: int, task_id: int = 0):
    """使用PNC服务进行规划导航到目标点"""
    url = (
        "http://10.42.10.11:53176/rpc/aimdk.protocol.PncService/PlanningNaviToGoal"
    )
    headers = {"Content-Type": "application/json"}
    data = {
        "header": {
            "timestamp": {"seconds": 0, "nanos": 0, "ms_since_epoch": 0},
            "control_source": 0,
        },
        "task_id": task_id,
        "map_id": map_id,
        "target_id": target_id,
        "guide_line_id": 0,
        "ackerman_mode": False,
    }
    rsp = requests.post(url, headers=headers, json=data)
    rsp.raise_for_status()
    response_data = rsp.json()
    if response_data.get("state") != "CommonState_SUCCESS":
        raise Exception(f"导航任务下发失败: {response_data.get('message', '未知错误')}")
    return response_data.get("task_id", task_id)


@retry(max_attempts=2, delay=0.5)
def get_pnc_task_state(task_id: int):
    """获取PNC任务状态"""
    url = "http://10.42.10.11:53176/rpc/aimdk.protocol.PncService/ActionGetState"
    headers = {"Content-Type": "application/json"}
    data = {
        "header": {
            "timestamp": {"seconds": 0, "nanos": 0, "ms_since_epoch": 0},
            "control_source": 0,
        },
        "task_id": task_id,
    }
    rsp = requests.post(url, headers=headers, json=data)
    rsp.raise_for_status()
    return rsp.json()["state"]


@retry(max_attempts=2, delay=0.5)
def cancel_pnc_task(task_id: int):
    """取消PNC任务"""
    url = "http://10.42.10.11:53176/rpc/aimdk.protocol.PncService/ActionCancel"
    headers = {"Content-Type": "application/json"}
    data = {
        "header": {
            "timestamp": {"seconds": 0, "nanos": 0, "ms_since_epoch": 0},
            "control_source": 0,
        },
        "task_id": task_id,
    }
    rsp = requests.post(url, headers=headers, json=data)
    rsp.raise_for_status()
    response_data = rsp.json()
    if response_data.get("state") != "CommonState_SUCCESS":
        raise Exception(f"任务取消失败: {response_data.get('message', '未知错误')}")
    return True


if __name__ == "__main__":
    # 获取当前地图ID
    current_map_id = get_current_map_id()
    print(f"当前地图ID: {current_map_id}")

    # 获取当前地图的导航点
    navi_points = get_navi_points(current_map_id)
    print(f"当前地图的导航点: {navi_points}")

    # 选择导航点
    for i, point in enumerate(navi_points):
        print(f"{i}: {point}")
    choice = input("请选择导航点: ")
    if choice.isdigit() and int(choice) >= 0 and int(choice) < len(navi_points):
        navi_point = navi_points[int(choice)]
        print(f"选择的导航点: {navi_point}")
    else:
        print("无效的选择")
        exit(1)
    target_id = navi_point["point_id"]
    print(f"选择的导航点ID: {target_id}")

    # 检查并切换到强化力控导航状态
    current_action = get_action()
    if current_action != "McAction_RL_LOCOMOTION_DEFAULT":
        print("请切换到强化力控站立姿态，再执行该导航任务")
        exit(1)
    if not ensure_action("McAction_RL_LOCOMOTION_DEFAULT"):
        print("切换到强化力控导航状态失败")
        exit(1)

    # 执行导航任务 - 使用新的PNC服务
    try:
        task_id = planning_navi_to_goal(int(current_map_id), int(target_id))
        print(f"导航任务已下发，任务ID: {task_id}")
    except Exception as e:
        print(f"导航任务下发失败: {e}")
        exit(1)

    # 等待导航任务完成
    try:
        while True:
            pnc_state = get_pnc_task_state(task_id)
            print(f"当前PNC任务状态: {pnc_state}")

            if pnc_state == "PncServiceState_SUCCESS":
                print("导航任务完成")
                break
            elif pnc_state == "PncServiceState_FAILED":
                print("导航任务失败")
                break
            elif pnc_state == "PncServiceState_RUNNING":
                print("导航任务运行中...")
            elif pnc_state == "PncServiceState_PAUSED":
                print("导航任务已暂停")
            elif pnc_state == "PncServiceState_IDLE":
                print("PNC服务空闲")
                break

            time.sleep(1)
    except KeyboardInterrupt:
        print("\n收到中断信号，正在取消导航任务...")
        try:
            cancel_pnc_task(task_id)
            print("导航任务已取消")
        except Exception as e:
            print(f"取消任务失败: {e}")
    except Exception as e:
        print(f"监控任务状态时出错: {e}")
