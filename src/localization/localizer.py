from db.device_localization import Device_localization
from db.localizationDAO import LocalizationDAO

class Localizer:
    def __init__(self, localizationDAO: LocalizationDAO):
        self.dao = localizationDAO
    
    def localize(self, source_id, local_coords):
        device_localization = self.dao.get_mapped_device_coordinates(source_id)
        return self.localize_calc(device_localization, local_coords)

    def localize_calc(self, device_localization, local_coords):
        self.get_correct_direction(device_localization, local_coords)
        start_coords = device_localization.device_location
        final_coordinates = {
            "x": local_coords['x'] + start_coords['x'],
            "y": local_coords['y'] + start_coords['y'],
            "z": local_coords['z'] + start_coords['z'],
        }
        return final_coordinates

    def get_correct_direction(self, device_localization: Device_localization, local_coords):
        if device_localization.x_invert:
            local_coords['x'] = -1 * local_coords['x']
        if device_localization.y_invert:
            local_coords['y'] = -1 * local_coords['y']
        if device_localization.z_invert:
            local_coords['z'] = -1 * local_coords['z']
        return local_coords
