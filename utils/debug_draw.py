import carla


def update_spectator(vehicle, spectator):
    transform = vehicle.get_transform()
    spectator.set_transform(
        carla.Transform(
            transform.location + carla.Location(z=40),
            carla.Rotation(pitch=-90)
        )
    )


def draw_route(world, route, life_time=30.0):
    if not route:
        print("No route found.")
        return

    for i, wp in enumerate(route):
        loc = wp.transform.location + carla.Location(z=0.5)

        if i == 0:
            color = carla.Color(255, 0, 0)
            size = 0.16
        elif i == len(route) - 1:
            color = carla.Color(0, 255, 0)
            size = 0.16
        else:
            color = carla.Color(255, 255, 0)
            size = 0.12

        world.debug.draw_point(
            loc,
            size=size,
            color=color,
            life_time=life_time
        )

        if i < len(route) - 1:
            next_loc = route[i + 1].transform.location + carla.Location(z=0.5)
            world.debug.draw_line(
                loc,
                next_loc,
                thickness=0.08,
                color=carla.Color(0, 0, 255),
                life_time=life_time
            )