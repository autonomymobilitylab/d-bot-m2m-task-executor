from db.postgres_connector import PostgresConnector
from db.device_localization import Device_localization

class LocalizationDAO:
	def __init__(self, db: PostgresConnector):
		self.conn = db

	def get_mapped_device_coordinates(self, device_id) -> Device_localization:
		res = self.conn.fetch(f"select * from public.device_localization where id = {device_id}")[0]
		dev_loc = Device_localization(res[0], res[1], res[2], res[3], res[4], res[5], res[6], res[7])
		return dev_loc