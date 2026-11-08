"""Process signals request a drain before the ROS context is destroyed."""

class ShutdownRequested(BaseException):
    """Only our process signal requests normal lifecycle shutdown."""


class ShutdownSignals:
    """Keep the ROS context live while active callbacks and final writes drain.

    Unlike rclpy's automatic signal handler, this never shuts down a context.
    Repeated signals during cleanup cannot interrupt the owned-resource drain.
    Runtime callback exceptions remain errors, including RuntimeError.
    """
    def __enter__(self):
        import signal
        self.cleaning = False
        self.requested = False
        self.previous = {sig: signal.signal(sig, self.request)
                         for sig in (signal.SIGINT, signal.SIGTERM)}
        return self

    def request(self, signum, frame):
        self.requested = True
        if not self.cleaning:
            self.cleaning = True
            raise ShutdownRequested()

    def begin_cleanup(self):
        self.cleaning = True

    def __exit__(self, *exception):
        import signal
        for sig, handler in self.previous.items():
            signal.signal(sig, handler)
