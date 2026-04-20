import carla
import random
import time
from perception.camera_manager import CameraManager
from perception.road_perception import RoadPerception
import cv2
from planning.global_planner import CarlaGraphPlanner
from control.controller import RouteFollower
from utils.debug_draw import draw_route, update_spectator
import config
camera_manager = None

def spawn_vehicle(world, start_point):
    blueprints = world.get_blueprint_library()
    vehicle_bp = blueprints.find(config.VEHICLE_BP)

    vehicle = world.try_spawn_actor(vehicle_bp, start_point)
    if vehicle is None:
        print("Spawn failed.")
        return None

    return vehicle


def main():
    client = carla.Client(config.HOST, config.PORT)
    client.set_timeout(config.TIMEOUT)

    world = client.get_world()
    carla_map = world.get_map()
    spectator = world.get_spectator()

    original_settings = world.get_settings()
    vehicle = None

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        planner = CarlaGraphPlanner(
            carla_map,
            sampling_resolution=2.0,
            lane_change_cost_factor=1.5,
            uturn_cost_factor=5.0,
            enable_lane_change=True,
            enable_uturn=True,
        )

        print("Planner summary:", planner.debug_summary())

        spawn_points = carla_map.get_spawn_points()
        if len(spawn_points) < 2:
            print("Not enough spawn points.")
            return

        start_point = random.choice(spawn_points)
        goal_point = random.choice(spawn_points)

        while goal_point.location.distance(start_point.location) < 5.0:
            goal_point = random.choice(spawn_points)

        print("Start:", start_point.location)
        print("Goal :", goal_point.location)

        route = planner.plan(
            start_point.location,
            goal_point.location
        )

        if not route:
            print("No route found.")
            return

        print("Route length:", len(route))

        draw_route(world, route, life_time=60.0)

        world.debug.draw_point(
            start_point.location + carla.Location(z=1.0),
            size=0.2,
            color=carla.Color(255, 0, 255),
            life_time=60.0
        )
        world.debug.draw_point(
            goal_point.location + carla.Location(z=1.0),
            size=0.2,
            color=carla.Color(0, 255, 255),
            life_time=60.0
        )

        vehicle = spawn_vehicle(world, start_point)
        if vehicle is None:
            return

        world.tick()

        controller = RouteFollower(
            vehicle,
            world,
            target_speed=config.TARGET_SPEED,
            steer_k=1.0,
            steer_soft_term=1.0,
            speed_kp=0.18,
            waypoint_reach_dist=config.WAYPOINT_REACH_DIST,
        )

        controller.set_route(route)
        camera_manager = CameraManager(
            world,
            world.get_blueprint_library(),
            vehicle,
            image_w=config.IMAGE_W,
            image_h=config.IMAGE_H,
            fov=config.CAMERA_FOV,
        )
        camera_manager.setup()

        road_perception = RoadPerception()

        for _ in range(3000):
            world.tick()

            control = controller.run_step()
            vehicle.apply_control(control)

            update_spectator(vehicle, spectator)

            rgb_image = camera_manager.get_rgb()
            semantic_image = camera_manager.get_semantic()

            if rgb_image is not None:
                rgb_vis = road_perception.carla_image_to_bgr_array(rgb_image)
                cv2.imshow("RGB", rgb_vis)

            if semantic_image is not None:
                semantic_vis = road_perception.semantic_to_cityscapes(semantic_image)
                drivable_mask = road_perception.get_drivable_mask_from_semantic(semantic_image)

                cv2.imshow("Semantic", semantic_vis)
                cv2.imshow("Drivable Mask", drivable_mask)

            key = cv2.waitKey(1)
            if key == 27:
                break

            time.sleep(0.02)

            if controller.is_finished():
                print("Route finished.")
                break

        time.sleep(2.0)

    finally:
        world.apply_settings(original_settings)

        if vehicle is not None:
            vehicle.destroy()
        if camera_manager is not None:
            camera_manager.destroy()

        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()