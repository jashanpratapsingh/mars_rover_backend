def init(args=None):
    pass

def spin(node):
    import time
    try:
        while True:
            time.sleep(0.1)
            # In a real mock, we might trigger callbacks here
            # For now, the mock publisher uses a timer which we need to simulate
            if hasattr(node, '_timers'):
                for timer in node._timers:
                    timer._tick()
    except KeyboardInterrupt:
        pass

def shutdown():
    pass
