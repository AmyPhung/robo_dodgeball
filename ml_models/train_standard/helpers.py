import os

def get_ws_path():
    home_path = os.path.expanduser("~") + "/"
    if home_path.split("/")[1] == "aphung":
        ws_name = "robo_ws"
    else:
        ws_name = "catkin_ws"
    ws_path = home_path + ws_name + "/src/ml_comprobofinal/"
    return ws_path