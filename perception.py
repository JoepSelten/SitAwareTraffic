

class Perception():
    def __init__(self):
        pass

    def recognize_sit(self, g):
        traffic_sign = "Road"
        # here the querying should be
        q = """
            PREFIX ex: <http://example.com/>
            
            SELECT ?part
            WHERE {
                ex:road ex:has_a ?part .
            }"""
        for r in g.query(q):
            print(r)

    def configure_sit(self, world):
        road_width = 20
        road_yaw = 0
        if world.situation == "Road":
            world.config_sit(road_width, road_yaw)

    def plot_raw_data(self, sim):
        pass
        