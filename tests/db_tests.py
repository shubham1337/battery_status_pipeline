
import os
import unittest
import datetime as dt
from unittest.mock import Mock

from common.db_schema import SATableBase, BatteryStatusLog


class TestDBSchema(unittest.TestCase):
    ''' Test Cases for testing various Database module '''

    def test_init_failure(self):
        ''' Test raising exception when initializing database '''

        with self.assertRaises(Exception):
            SATableBase('')

    def test_init_success(self):
        ''' Test successfull initializing of database '''

        bsl: BatteryStatusLog = BatteryStatusLog('sqlite://')
        # This will run a select query on the database,
        # if the database was correctly initialized this will work
        bsl.select_all().fetchall()

    def test_insert(self):
        ''' Test inserting rows into the table '''

        mock_data: dict = {
            'timestamp': dt.datetime.now(),
            'current': 0.1,
            'voltage': 0.1,
            'power_supply_status': 1,
        }
        bsl: BatteryStatusLog = BatteryStatusLog('sqlite://')
        bsl.insert(mock_data)
        return_list: list = list(bsl.select_all().fetchall())
        self.assertDictEqual(dict(return_list[0]), mock_data)

    def test_failure_insert(self):
        ''' Test failure when inserting rows into the table '''

        mock_data: dict = {
            'timestamp': dt.datetime.now(),
            'current': 0.1,
            'voltage': 0.1,
            'power_supply_status': 1,
        }
        bsl: BatteryStatusLog = BatteryStatusLog('sqlite://')
        bsl.insert = Mock()
        bsl.insert.side_effect = Exception('InsertionException')
        with self.assertRaises(Exception):
            bsl.insert(mock_data)
        # No data was inserted
        return_list: list = list(bsl.select_all().fetchall())
        self.assertEqual(len(return_list), 0)

if __name__ == '__main__':
    unittest.main()
