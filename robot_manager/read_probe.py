"""Bounded retry of a read-only health request; never used for mutations."""
import time


class ReadProbe:
    def __init__(self, client, request_factory, clock=time.monotonic, retry_seconds=1.0):
        self.client, self.request_factory = client, request_factory
        self.clock, self.retry_seconds = clock, retry_seconds
        self.future = None
        self.sent_at = 0
        self.result = None

    def poll(self):
        now = self.clock()
        if self.future is not None:
            if self.future.done():
                try:
                    self.result = self.future.result()
                except Exception:
                    self.result = None
                self.future = None
            elif now - self.sent_at >= self.retry_seconds:
                self.future.cancel()
                self.future = None
        if self.result is None and self.future is None and self.client.service_is_ready():
            self.future = self.client.call_async(self.request_factory())
            self.sent_at = now
        return self.result
