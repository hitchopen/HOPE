"""E-link hand presets used by the asynchronous serve bridge.

Declarative data only: importing this module never opens a transport.
"""
CMD_TOPIC = '/body_drive/hand_joint_command/pb_3Aaimdk_2Eprotocol_2EHandCommandChannel'
PRESETS = {
    "grab": {"method": 1, "pos": 800, "force": 20, "vel": 40},
    "release": {"method": 1, "pos": 2000, "force": 20, "vel": 60},
}
