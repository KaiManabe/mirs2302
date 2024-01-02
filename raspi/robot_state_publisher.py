import json


def update(key, value):
    with open("/home/pi/git/mirs2302/raspi/robot_status.json", "r") as f:
        current = json.load(f)
    current[key] = value
    with open("/home/pi/git/mirs2302/raspi/robot_status.json", "w") as f:
        json.dump(current, f)

def current_status():
    with open("/home/pi/git/mirs2302/raspi/robot_status.json", "r") as f:
        current = json.load(f)
    return current