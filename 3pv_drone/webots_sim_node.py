import os

def start_webots_sim():
    """ Begin webots simulation via terminal command """

    webots_world_path = os.path.join(  # construct path to find webots world file
        "/", *os.path.dirname(__file__).split("/")[1:-6], "src/3pv_drone/simulation/worlds/3pv_testing.wbt")

    os.system(f"webots {webots_world_path}")  # start sim

    # print status
    print(f"webots simulation started from world file at {webots_world_path}")

def main():
    start_webots_sim()