import carla
import queue


class CameraManager:
    def __init__(self, world, blueprint_library, vehicle, image_w=640, image_h=480, fov=90):
        self.world = world
        self.blueprint_library = blueprint_library
        self.vehicle = vehicle
        self.image_w = image_w
        self.image_h = image_h
        self.fov = fov

        self.rgb_camera = None
        self.semantic_camera = None

        self.rgb_queue = queue.Queue()
        self.semantic_queue = queue.Queue()

    def setup(self):
        camera_transform = carla.Transform(
            carla.Location(x=1.8, z=1.4),
            carla.Rotation(pitch=-10)
        )

        self.rgb_camera = self._spawn_camera(
            "sensor.camera.rgb",
            camera_transform
        )
        self.semantic_camera = self._spawn_camera(
            "sensor.camera.semantic_segmentation",
            camera_transform
        )

        self.rgb_camera.listen(lambda image: self.rgb_queue.put(image))
        self.semantic_camera.listen(lambda image: self.semantic_queue.put(image))

    def _spawn_camera(self, sensor_type, transform):
        bp = self.blueprint_library.find(sensor_type)
        bp.set_attribute("image_size_x", str(self.image_w))
        bp.set_attribute("image_size_y", str(self.image_h))
        bp.set_attribute("fov", str(self.fov))
        return self.world.spawn_actor(bp, transform, attach_to=self.vehicle)

    def get_rgb(self):
        if self.rgb_queue.empty():
            return None
        return self.rgb_queue.get()

    def get_semantic(self):
        if self.semantic_queue.empty():
            return None
        return self.semantic_queue.get()

    def destroy(self):
        if self.rgb_camera is not None:
            self.rgb_camera.stop()
            self.rgb_camera.destroy()
        if self.semantic_camera is not None:
            self.semantic_camera.stop()
            self.semantic_camera.destroy()