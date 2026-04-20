import numpy as np
import carla


class RoadPerception:
    ROAD_LABEL = 7

    @staticmethod
    def carla_image_to_bgr_array(image):
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        return array[:, :, :3]

    @staticmethod
    def semantic_to_cityscapes(image):
        image.convert(carla.ColorConverter.CityScapesPalette)
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        return array[:, :, :3]
    @staticmethod
    def semantic_raw_to_labels(image):
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        labels = array[:, :, 2]
        return labels

    def get_drivable_mask_from_semantic(self, semantic_image):
        labels = self.semantic_raw_to_labels(semantic_image)
        mask = (labels == self.ROAD_LABEL).astype(np.uint8) * 255
        return mask