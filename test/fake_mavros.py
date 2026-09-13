"""A stand-in for MAVROS's /mavros/cmd/command client, for payload tests."""

import math
from types import SimpleNamespace


class FakeFuture:
    def __init__(self, client):
        self.client = client
        self.callback = None
        self.response = None
        self.error = None

    def add_done_callback(self, callback):
        self.callback = callback
        if self.client.auto_reply is not None:
            self.finish(**self.client.auto_reply)

    def finish(self, success=True, result=0, error=None):
        self.response = SimpleNamespace(success=success, result=result)
        self.error = error
        self.callback(self)

    def result(self):
        if self.error is not None:
            raise self.error
        return self.response


class FakeCommandClient:
    """Replies at once (success) unless auto_reply is changed or None.

    With auto_reply None, a test finishes futures itself to control exactly
    when PX4's reply arrives.
    """

    def __init__(self):
        self.requests = []
        self.futures = []
        self.auto_reply = {'success': True}

    def call_async(self, request):
        self.requests.append(request)
        future = FakeFuture(self)
        self.futures.append(future)
        return future


class Clock:
    def __init__(self, now=1000.0):
        self.now = now

    def __call__(self):
        return self.now


def decode(value, low=800, high=2200):
    """Pulse (us) from a PX4 actuator value, None for 'unchanged' (NaN)."""
    return None if math.isnan(value) else round(
        low + (value + 1.0) / 2.0 * (high - low))


def sent(client):
    """(plate_us, clamp_us) per command. Clamp is set 1, plate set 2."""
    return [(decode(r.param2), decode(r.param1)) for r in client.requests]
