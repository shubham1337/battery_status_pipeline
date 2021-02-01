
import os
import unittest
from unittest.mock import patch, call, Mock

# Set both databases to in-memory databases for tests
os.environ['SQLITE_DB_PATH'] = 'sqlite://'
os.environ['CLOUD_DB_PATH'] = 'sqlite://'


class TestCloudSync(unittest.TestCase):
    ''' Test Cases for testing various cases for CloudSync module '''

    @patch('common.db_schema.BatteryStatusLog')
    def test_db_initialized(self, MockBatteryStatusLog):
        ''' Test Successful initialization of databases '''

        from cloud_sync.cloud_sync import CloudSyncTask
        # MockBatteryStatusLog Must be called when initializing both local and cloud databases
        CloudSyncTask()
        MockBatteryStatusLog.assert_has_calls([call('sqlite://'), call('sqlite://')])

    def test_db_not_initialized(self):
        ''' Raise exception for invalid db urls '''

        os.environ['SQLITE_DB_PATH'] = ''
        os.environ['CLOUD_DB_PATH'] = ''
        with self.assertRaises(Exception):
            CloudSyncTask()


    @patch('common.db_schema.BatteryStatusLog')
    def test_task_failure(self, MockBatteryStatusLog):
        ''' Test if rollback is called when there is an exception when running the task '''

        from cloud_sync.cloud_sync import CloudSyncTask
        cst: CloudSyncTask = CloudSyncTask()
        cst.cloud_db.table.insert.side_effect = Exception('Network Error')
        with self.assertRaises(Exception):
            cst.run()
        cst.cloud_db.connection.begin.assert_called()
        cst.local_db.select_all.assert_called()
        cst.local_db.select_all().fetchall.assert_called()
        with cst.cloud_db.connection.begin() as transaction:
            transaction.rollback.assert_called()


    @patch('common.db_schema.BatteryStatusLog')
    def test_task_run(self, MockBatteryStatusLog):
        ''' Test all database calls when running the task '''

        from cloud_sync.cloud_sync import CloudSyncTask
        cst: CloudSyncTask = CloudSyncTask()
        cst.cloud_db.table.insert.side_effect = None
        cst.run()
        cst.cloud_db.connection.begin.assert_called()
        cst.local_db.select_all.assert_called()
        cst.local_db.select_all().fetchall.assert_called()
        cst.cloud_db.connection.execute.assert_has_calls([
            call(cst.cloud_db.table.insert(), cst.local_db.select_all().fetchall()),
            call(cst.cloud_db.table.delete()),
        ])
        with cst.cloud_db.connection.begin() as transaction:
            transaction.commit.assert_called()

if __name__ == '__main__':
    unittest.main()
