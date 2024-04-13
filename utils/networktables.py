from __future__ import annotations
import ntcore
from enum import Enum

class NetworkTableManager:
    
    def __init__(self):
        self.nt = ntcore.NetworkTableInstance.getDefault()
        
    def getTable(self, table_name: str) -> NetworkTable:
        table = self.nt.getTable(table_name)
        return NetworkTable(table)

class NetworkTable:
    
    class TopicType(Enum):
        DOUBLE = 0
        STRING = 1
        BOOLEAN = 2
        RAW = 3
        BOOLEAN_ARRAY = 4
        DOUBLE_ARRAY = 5
        STRING_ARRAY = 6
        
    entries: dict[str,
            ntcore.BooleanArrayEntry | ntcore.BooleanEntry | ntcore.RawEntry | ntcore.DoubleEntry | ntcore.DoubleArrayEntry
            ]={}
    
    def __init__(self, table_name: str | ntcore.NetworkTable):
        if isinstance(table_name, str):
            
            self.table = ntcore.NetworkTableInstance.getDefault().getTable(table_name)
            self.name = self.table.getPath()
        elif isinstance(table_name, ntcore.NetworkTable):
            self.table = table_name
            self.name = self.table.getPath()
        else:
            raise ValueError("Invalid table_name type")
        
    def getNumber(self, key: str, default: float = 0.0) -> float:
        
        entry:ntcore.DoubleEntry = self.__get_entry(key, NetworkTable.TopicType.DOUBLE, default)
        
        return entry.get(default)
    
    def getString(self, key: str, default: str = "") -> str:
        
        entry:ntcore.StringEntry = self.__get_entry(key, NetworkTable.TopicType.STRING, default)
        
        return entry.get(default)
    
    def getBoolean(self, key: str, default: bool = False) -> bool:
        
        entry:ntcore.BooleanEntry = self.__get_entry(key, NetworkTable.TopicType.BOOLEAN, default)
        
        return entry.get(default)
    
    def getRaw(self, key: str, default: bytes = b"") -> bytes:
        
        entry:ntcore.RawEntry = self.__get_entry(key, NetworkTable.TopicType.RAW, default)
        
        return entry.get(default)
    
    def getBooleanArray(self, key: str, default: list[bool] = []) -> list[bool]:
        
        entry:ntcore.BooleanArrayEntry = self.__get_entry(key, NetworkTable.TopicType.BOOLEAN_ARRAY, default)
        
        return entry.get(default)
    
    def getNumberArray(self, key: str, default: list[float] = []) -> list[float]:
        
        entry:ntcore.DoubleArrayEntry = self.__get_entry(key, NetworkTable.TopicType.DOUBLE_ARRAY, default)
        
        return entry.get(default)
    
    def getStringArray(self, key: str, default: list[str] = []) -> list[str]:
        
        entry:ntcore.StringArrayEntry = self.__get_entry(key, NetworkTable.TopicType.STRING_ARRAY, default)
        
        return entry.get(default)
    
    def putNumber(self, key: str, value: float):
        
        entry:ntcore.DoubleEntry = self.__get_entry(key, NetworkTable.TopicType.DOUBLE, value)
        
        entry.set(value)
        
    def putString(self, key: str, value: str):
        
        entry:ntcore.StringEntry = self.__get_entry(key, NetworkTable.TopicType.STRING, value)
        
        entry.set(value)
        
    def putBoolean(self, key: str, value: bool):
        
        entry:ntcore.BooleanEntry = self.__get_entry(key, NetworkTable.TopicType.BOOLEAN, value)
        
        entry.set(value)
        
    def putRaw(self, key: str, value: bytes):
        
        entry:ntcore.RawEntry = self.__get_entry(key, NetworkTable.TopicType.RAW, value)
        
        entry.set(value)
        
    def putBooleanArray(self, key: str, value: list[bool]):
        
        entry:ntcore.BooleanArrayEntry = self.__get_entry(key, NetworkTable.TopicType.BOOLEAN_ARRAY, value)
        
        entry.set(value)
        
    def putNumberArray(self, key: str, value: list[float]):
        
        entry:ntcore.DoubleArrayEntry = self.__get_entry(key, NetworkTable.TopicType.DOUBLE_ARRAY, value)
        
        entry.set(value)
        
    def putStringArray(self, key: str, value: list[str]):
        
        entry:ntcore.StringArrayEntry = self.__get_entry(key, NetworkTable.TopicType.STRING_ARRAY, value)
        
        entry.set(value)
        
    def getSubTable(self, key: str) -> NetworkTable:

        return NetworkTable(self.table.getSubTable(key))    
    
        
    def __get_entry(self, key: str, type: TopicType, default: any):
        
        entry = None
        
        entry = self.entries.get(self.name+key)
        
        if entry is None:
            print(self.name, "Creating new entry", self.name+key, type, default)
            entry = self.__create_entry(key, type, default)
            print(self.name, 'entries size', len(self.entries))
        
        return entry
        
        
    def __create_entry(self, key: str, type: TopicType, default: any):
        
        topic = None
        
        match type:
            case NetworkTable.TopicType.DOUBLE:
                topic = self.table.getDoubleTopic(key)
            case NetworkTable.TopicType.STRING:
                topic = self.table.getStringTopic(key)
            case NetworkTable.TopicType.BOOLEAN:
                topic = self.table.getBooleanTopic(key)
            case NetworkTable.TopicType.RAW:
                topic = self.table.getRawTopic(key)
            case NetworkTable.TopicType.BOOLEAN_ARRAY:
                topic = self.table.getBooleanArrayTopic(key)
            case NetworkTable.TopicType.DOUBLE_ARRAY:
                topic = self.table.getDoubleArrayTopic(key)
            case NetworkTable.TopicType.STRING_ARRAY:
                topic = self.table.getStringArrayTopic(key)
            case _:
                raise ValueError("Invalid TopicType")
        
        entry = topic.getEntry(default)
        
        self.entries[self.name+key] = entry
        
        return entry
    
    
    