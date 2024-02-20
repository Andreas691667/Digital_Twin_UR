class MSG_TYPES:
    """Types of messages to be sent from DT to Controller"""
    MONITOR = 0             # Used for monitor messages
    WAIT = 1                # Used for DT->PT messages before fault have been resolved
    RESOLVED = 2            # Used to send resolution 
    COULD_NOT_RESOLVE = 3   # Used to send message where fault could not be resolved
    



class MSG_TOPICS:
    """Topic conversion between UR and DT and Controller and UR
    Contains names and indexes of the topics as a dictionary"""

    TIMESTAMP = ["timestamp", 0]
    START_BIT = ["output_bit_register_65", 7]
    OBJECT_DETECTED = ["output_bit_register_66", 8]

    W1 = [
        "input_double_register_24",
        "input_double_register_25",
        "input_double_register_26",
        "input_double_register_27",
        "input_double_register_28",
        "input_double_register_29",
    ]

    W2 = [
        "input_double_register_30",
        "input_double_register_31",
        "input_double_register_32",
        "input_double_register_33",
        "input_double_register_34",
        "input_double_register_35",
    ]

    W3 = [
        "input_double_register_36",
        "input_double_register_37",
        "input_double_register_38",
        "input_double_register_39",
        "input_double_register_40",
        "input_double_register_41",
    ]

    W4 = [
        "input_double_register_42",
        "input_double_register_43",
        "input_double_register_44",
        "input_double_register_45",
        "input_double_register_46",
        "input_double_register_47",
    ]

    GP = ["input_int_register_24"]
