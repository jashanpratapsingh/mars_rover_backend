import time
import sys

def _get_bus():
    if 'mock_ros_bus' not in sys.modules:
        sys.modules['mock_ros_bus'] = {}
    return sys.modules['mock_ros_bus']

class Node:
    def __init__(self, node_name):
        self.node_name = node_name
        self._timers = []
        self._subs = []
        self._pubs = []

    def create_subscription(self, msg_type, topic, callback, qos_profile):
        sub = MockSubscription(topic, callback)
        self._subs.append(sub)
        
        bus = _get_bus()
        if topic not in bus:
            bus[topic] = []
        bus[topic].append(callback)
        
        return sub

    def create_publisher(self, msg_type, topic, qos_profile):
        pub = MockPublisher(topic)
        self._pubs.append(pub)
        return pub

    def create_timer(self, interval, callback):
        timer = MockTimer(interval, callback)
        self._timers.append(timer)
        return timer

    def get_logger(self):
        return MockLogger()

    def destroy_node(self):
        pass

class MockSubscription:
    def __init__(self, topic, callback):
        self.topic = topic
        self.callback = callback

class MockPublisher:
    def __init__(self, topic):
        self.topic = topic

    def publish(self, msg):
        bus = _get_bus()
        if self.topic in bus:
            for callback in bus[self.topic]:
                try:
                    callback(msg)
                except Exception as e:
                    print(f"Error in mock callback: {e}")

class MockTimer:
    def __init__(self, interval, callback):
        self.interval = interval
        self.callback = callback
        self.last_call = time.time()

    def _tick(self):
        now = time.time()
        if now - self.last_call >= self.interval:
            self.callback()
            self.last_call = now

class MockLogger:
    def info(self, msg):
        print(f"[INFO] {msg}")
    def warn(self, msg):
        print(f"[WARN] {msg}")
    def error(self, msg):
        print(f"[ERROR] {msg}")
