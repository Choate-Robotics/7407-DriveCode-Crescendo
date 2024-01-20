from utils import LocalLogger

log = LocalLogger("IT")


class IT:

    @staticmethod
    def init() -> None:
        log.info("Initializing IT...")

    @staticmethod
    def map_systems():
        log.info("Mapping systems...")
        pass
