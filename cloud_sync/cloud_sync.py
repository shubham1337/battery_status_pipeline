
import os
import datetime as dt
from common.db_schema import BatteryStatusLog

# For type hinting
from sqlalchemy.engine.result import RowProxy

# Initialize Environment variables
SQLITE_DB_PATH: str = os.getenv('SQLITE_DB_PATH')
CLOUD_DB_PATH: str = os.getenv('CLOUD_DB_PATH')


class CloudSyncTask:
    ''' A simple task which syncs local database with the cloud database '''

    def __init__(self) -> None:
        self.local_db: BatteryStatusLog = BatteryStatusLog(SQLITE_DB_PATH)
        self.cloud_db: BatteryStatusLog = BatteryStatusLog(CLOUD_DB_PATH)

    def run(self) -> None:
        ''' Entry function for the task '''

        # If there is any error when inserting rows to cloud_db then rollback the transaction
        with self.cloud_db.connection.begin() as transaction:
            try:
                # Fetch rows from local database
                rows: list[RowProxy] = self.local_db.select_all().fetchall()

                # Insert the above fetched rows to cloud database
                self.cloud_db.connection.execute(self.cloud_db.table.insert(), rows)

                # Removes rows from local database after successful sync
                delete_all_rows: str = self.local_db.table.delete()
                self.local_db.connection.execute(delete_all_rows)
            except:
                transaction.rollback()
                raise
            else:
                transaction.commit()

if __name__ == '__main__':
    CloudSyncTask().run()
