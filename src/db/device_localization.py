
class Device_localization:
    def __init__(self, dev_id, info, device_location, x_invert, y_invert, z_invert,created_timestamp, modified_timestamp):
        self.dev_id = dev_id
        self.info = info
        self.device_location = device_location
        self.x_invert = x_invert
        self.y_invert = y_invert
        self.z_invert = z_invert
        self.created_timestamp = created_timestamp
        self.modified_timestamp = modified_timestamp

    def stringify(self):
        data_dict = self.__dict__
        return ', '.join([f"{key}='{value}'" for key, value in data_dict.items()])
