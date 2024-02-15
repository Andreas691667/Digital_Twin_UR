class MSG_TYPES:
    """Types of messages to be sent from DT to Controller"""
    STOP_PROGRAM = "STOP_PROGRAM"

class MSG_TOPICS:
    """Topic conversion between UR and DT
    Contains names and indexes of the topics as a dictionary"""
    TIMESTAMP = ["timestamp", 0]
    START_BIT = ["output_bit_register_65", 7]
    OBJECT_DETECTED = ["output_bit_register_66", 8]