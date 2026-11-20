# The logger keeps Bluetooth off. Skip the shared BLE host sources.
Import("env")


def skip_idle_ble(node):
    path = str(node).replace("\\", "/")
    if path.endswith("/ble_api.cpp") or path.endswith("/bluedroid_api.cpp"):
        return None
    return node


env.AddBuildMiddleware(skip_idle_ble, "*api.cpp")
