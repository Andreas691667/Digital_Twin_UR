class MSG_TYPES_DT_TO_CONTROLLER:
    """Types of messages to be sent from DT to Controller"""
    MONITOR = "MONITOR"             # Used for monitor messages
    WAIT = "WAIT"                # Used for DT->PT messages before fault have been resolved
    RESOLVED = "RESOLVED"            # Used to send resolution 
    COULD_NOT_RESOLVE = "COULD_NOT_RESOLVE"   # Used to send message where fault could not be resolved
    FINISHED_TIMING = "FINISHED_TIMING"
    TASK_VALIDATED = "TASK_VALIDATED"
    TASK_NOT_VALIDATED = "TASK_NOT_VALIDATED"

class MSG_TYPES_MONITOR_TO_DT:
    """Types of messages to be sent from Monitor to DT"""
    MONITOR_DATA = "MONITOR_DATA"       # Used for monitor messages
class MSG_TYPES_CONTROLLER_TO_DT:
    """Types of messages to be sent from Controller to DT"""
    NEW_TASK = "NEW_TASK"            # Controller initiates a new task

class MSG_TOPICS:
    """Topic conversion between UR and DT and Controller
    Contains names and indexes of the topics as a dictionary"""
    pass