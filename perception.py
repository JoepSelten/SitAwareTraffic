

class Perception():
    def __init__(self):
        pass

    def recognize_sit(self, world):
        traffic_sign = "Road"
        world.update_sit(traffic_sign)

    def configure_sit(self, world):
        road_width = 20
        road_yaw = 0
        if world.situation == "Road":
            world.config_sit(road_width, road_yaw)
        