import ntcore
from enum import Enum


class NetworkTables:
    
    class TopicType(Enum):
        DOUBLE = 0
        STRING = 1
        BOOLEAN = 2
        RAW = 3
        BOOLEAN_ARRAY = 4
        DOUBLE_ARRAY = 5
        STRING_ARRAY = 6
    
    def __init__(self, table_name: str):
        self.nt = ntcore.NetworkTableInstance.getDefault()
        self.table = self.nt.getTable(table_name)
        self.entries: dict[str,
            ntcore.BooleanArrayEntry | ntcore.BooleanEntry | ntcore.RawEntry | ntcore.DoubleEntry | ntcore.DoubleArrayEntry
            ] = {}
        
    def getNumber(self, key: str, default: float = 0.0) -> float:
        
        entry:ntcore.DoubleEntry = self.__get_entry(key, NetworkTables.TopicType.DOUBLE, default)
        
        return entry.get(default)
    
    def getString(self, key: str, default: str = "") -> str:
        
        entry:ntcore.StringEntry = self.__get_entry(key, NetworkTables.TopicType.STRING, default)
        
        return entry.get(default)
    
    def getBoolean(self, key: str, default: bool = False) -> bool:
        
        entry:ntcore.BooleanEntry = self.__get_entry(key, NetworkTables.TopicType.BOOLEAN, default)
        
        return entry.get(default)
    
    def getRaw(self, key: str, default: bytes = b"") -> bytes:
        
        entry:ntcore.RawEntry = self.__get_entry(key, NetworkTables.TopicType.RAW, default)
        
        return entry.get(default)
    
    def getBooleanArray(self, key: str, default: list[bool] = []) -> list[bool]:
        
        entry:ntcore.BooleanArrayEntry = self.__get_entry(key, NetworkTables.TopicType.BOOLEAN_ARRAY, default)
        
        return entry.get(default)
    
    def getNumberArray(self, key: str, default: list[float] = []) -> list[float]:
        
        entry:ntcore.DoubleArrayEntry = self.__get_entry(key, NetworkTables.TopicType.DOUBLE_ARRAY, default)
        
        return entry.get(default)
    
    def getStringArray(self, key: str, default: list[str] = []) -> list[str]:
        
        entry:ntcore.StringArrayEntry = self.__get_entry(key, NetworkTables.TopicType.STRING_ARRAY, default)
        
        return entry.get(default)
    
    def putNumber(self, key: str, value: float):
        
        entry:ntcore.DoubleEntry = self.__get_entry(key, NetworkTables.TopicType.DOUBLE, value)
        
        entry.set(value)
        
    def putString(self, key: str, value: str):
        
        entry:ntcore.StringEntry = self.__get_entry(key, NetworkTables.TopicType.STRING, value)
        
        entry.set(value)
        
    def putBoolean(self, key: str, value: bool):
        
        entry:ntcore.BooleanEntry = self.__get_entry(key, NetworkTables.TopicType.BOOLEAN, value)
        
        entry.set(value)
        
    def putRaw(self, key: str, value: bytes):
        
        entry:ntcore.RawEntry = self.__get_entry(key, NetworkTables.TopicType.RAW, value)
        
        entry.set(value)
        
    def putBooleanArray(self, key: str, value: list[bool]):
        
        entry:ntcore.BooleanArrayEntry = self.__get_entry(key, NetworkTables.TopicType.BOOLEAN_ARRAY, value)
        
        entry.set(value)
        
    def putNumberArray(self, key: str, value: list[float]):
        
        entry:ntcore.DoubleArrayEntry = self.__get_entry(key, NetworkTables.TopicType.DOUBLE_ARRAY, value)
        
        entry.set(value)
        
    def putStringArray(self, key: str, value: list[str]):
        
        entry:ntcore.StringArrayEntry = self.__get_entry(key, NetworkTables.TopicType.STRING_ARRAY, value)
        
        entry.set(value)
        
    def __get_entry(self, key: str, type: TopicType, default: any):
        
        entry = None
        
        try:
            entry = self.entries[key]
        except KeyError:
            entry = self.__create_entry(key, type, default)
        
        return entry
        
        
    def __create_entry(self, key: str, type: TopicType, default: any):
        
        topic = None
        
        match type:
            case NetworkTables.TopicType.DOUBLE:
                topic = self.table.getDoubleTopic(key)
            case NetworkTables.TopicType.STRING:
                topic = self.table.getStringTopic(key)
            case NetworkTables.TopicType.BOOLEAN:
                topic = self.table.getBooleanTopic(key)
            case NetworkTables.TopicType.RAW:
                topic = self.table.getRawTopic(key)
            case NetworkTables.TopicType.BOOLEAN_ARRAY:
                topic = self.table.getBooleanArrayTopic(key)
            case NetworkTables.TopicType.DOUBLE_ARRAY:
                topic = self.table.getDoubleArrayTopic(key)
            case NetworkTables.TopicType.STRING_ARRAY:
                topic = self.table.getStringArrayTopic(key)
            case _:
                raise ValueError("Invalid TopicType")
        
        entry = topic.getEntry(default)
        
        self.topics[key] = entry
        
        return entry
    
    