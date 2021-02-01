

from sqlalchemy import create_engine, MetaData, Table, Column, Integer, Float, DateTime

# For type hinting
from sqlalchemy.engine.result import RowProxy
from sqlalchemy.engine import Engine
from sqlalchemy.engine.base import Connection


class SATableBase:
    ''' Base class which can be inherited for building SQL based tables using SQLAlchemy '''

    def __init__(self, db_path: str) -> None:
        if not (type(db_path) is str and len(db_path) > 0):
            raise Exception('Invalid db_path')
        self.meta: MetaData = MetaData()
        self.engine: Engine = create_engine(db_path, echo=True)
        self.table: Table = self.table_schema()
        self.create_table()
        self.connection: Connection = self.engine.connect()

    def create_table(self) -> None:
        ''' Executes all CREATE TABLE commands and builds the tables in the database '''
        self.meta.create_all(self.engine)

    def insert(self, row_dict: dict) -> RowProxy:
        ''' Utility function to insert one row in self.table '''
        return self.connection.execute(self.table.insert(), [row_dict])

    def select_all(self) -> RowProxy:
        ''' Utility function to query all rows from self.table '''
        sel: str = self.table.select()
        return self.connection.execute(sel)

    def table_schema(self) -> Table:
        ''' Must be implemented by child classes '''
        raise Exception('Not Implemented.')


class BatteryStatusLog(SATableBase):
    ''' Child class which is specific to BatteryStatusLog table '''

    def table_schema(self) -> Table:
        ''' Defines BatteryStatusLog's schema '''
        return Table(
           'battery_status_log', self.meta,
           Column('timestamp', DateTime), 
           Column('current', Float), 
           Column('voltage', Float),
           Column('power_supply_status', Integer),
        )
