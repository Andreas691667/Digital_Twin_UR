class RMQ_CONFIG():
    """RabbitMQ configuration"""
    RMQ_SERVER_IP = "localhost" #Andreas home:  10.0.0.228, Andreas drobotti: 192.168.0.165
    RMQ_SERVER_PORT = 5672
    MONITOR_EXCHANGE = "MONITOR_EXCHANGE"
    MONITOR_QUEUE = "MONITOR_QUEUE"
    DT_EXCHANGE = "DT_EXCHANGE"
    DT_QUEUE = "DT_QUEUE"   
    FANOUT = "fanout"
    DIRECT = "direct"
    