
from common.db_schema import BatteryStatusLog
import os
import datatime as dt

SQLITE_DB_PATH = os.getenv('SQLITE_DB_PATH')
CLOUD_DB_PATH = os.getenv('CLOUD_DB_PATH')

class CloudSyncTask():

    def __init__(self):
        self.local_db = BatteryStatusLog(SQLITE_DB_PATH)
        self.cloud_db = BatteryStatusLog(CLOUD_DB_PATH)

    def run(self):
        with self.cloud_db.connection.begin() as transaction:
            try:
                rows = self.local_db.select_all().fetchall()
                self.cloud_db.connection.execute(self.cloud_db.table.insert(), rows)
                delete_all_rows = self.local_db.table.delete()
                self.local_db.connection.execute(delete_all_rows)
            except:
                transaction.rollback()
                raise
            else:
                transaction.commit()

if __name__ == '__main__':
    CloudSyncTask().run()
