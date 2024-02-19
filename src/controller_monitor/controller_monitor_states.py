class CM_STATES:
    """Class for the states of the ControllerMonitor"""
    INITIALIZING = 0 # Before establishing connection to broker
    READY = 1 # Waits for user to press enter
    NORMAL_OPERATION = 2 # Performing task
    WAITING_FOR_DT = 3 # Waiting for resolution
    MANUEL_INTERVENTION = 4 # Waiting for manuel intervention