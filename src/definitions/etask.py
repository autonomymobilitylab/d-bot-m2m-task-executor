from enum import IntEnum, Enum
 
class ETask(IntEnum):
    NAVIGATE = 1
    STOP = 2
    WAIT = 3
    CHECK_AUTH_ILMATAR = 4
    CHECK_AUTH_KONE = 5
    CHECK_AUTH_NOCCELA = 6
    STATUS_ELEVATOR = 7
    STATUS_CRANE = 8
    CALL_ELEVATOR = 9
    POSITION_CRANE = 10