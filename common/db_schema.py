
from sqlalchemy import create_engine, MetaData, Table, Column, Integer, Float, DateTime


class SATableBase():

    def __init__(self, db_path):
        self.meta = MetaData()
        self.engine = create_engine(db_path, echo=True)
        self.table = self.table_schema()
        self.create_table()
        self.connection = self.engine.connect()

    def create_table(self):
        self.meta.create_all(self.engine)

    def insert(self, row_dict):
        return self.connection.execute(self.table.insert(), [row_dict])

    def select_all(self):
        sel = self.table.select()
        return self.connection.execute(sel)

    def table_schema(self):
        raise Exception('Not Implemented.')


class BatteryStatusLog(SATableBase):

    def table_schema(self):
        return Table(
           'battery_status_log', self.meta,
           Column('timestamp', DateTime), 
           Column('current', Float), 
           Column('voltage', Float),
           Column('power_supply_status', Integer),
        )
