import carla
from graph_builder import CarlaGraphBuilder

def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    carla_map = world.get_map()

    builder = CarlaGraphBuilder(
        carla_map=carla_map,
        sampling_resolution=2.0,
        lane_change_cost_factor=3,
        uturn_cost_factor=5.0,
        enable_lane_change=True,
        enable_uturn=True,
    )

    builder.build()
    print(builder.debug_summary())

    save_path = "D:/carla_pjt/town_graph.json"
    builder.save(save_path)
    print(f"graph saved to {save_path}")

if __name__ == "__main__":
    main()