import yaml


class WaypointManager:

    def __init__(self, config_path):

        with open(config_path, 'r') as f:
            self.waypoints = yaml.safe_load(f)

    def get_waypoint(self, name):

        return self.waypoints[name]