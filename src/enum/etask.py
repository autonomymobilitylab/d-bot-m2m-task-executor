from enum import Enum
 
class ETask(Enum):
    NAVIGATE = 1
    STOP = 2
    WAIT = 3
    CHECK_AUTH_ILMATAR = 4
    CHECK_AUTH_KONE = 5
    CHECK_AUTH_NOCCELA = 6
    STATUS_ELEVATOR = 7
    STATUS_CRANE = 8