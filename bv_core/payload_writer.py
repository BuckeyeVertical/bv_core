"""Send payload servo positions to PX4 through MAVROS.

The one piece of code that commands the plate and clamp: mission_node's DEPLOY
state and the test_servo bench tool both drive their drops through a
PayloadWriter, so a bench drop exercises exactly what flies.
"""

import time

from mavros_msgs.srv import CommandLong

from .payload import actuator_params

# Pause before re-sending a command PX4 did not confirm.
RETRY_SEC = 0.2


def actuator_request(config, plate_us=None, clamp_us=None):
    """CommandLong request putting the servos at these pulses (us).

    Addressed to PX4, not broadcast: PX4 on the flight controller answers a
    broadcast MAV_CMD_DO_SET_ACTUATOR with UNSUPPORTED and leaves the outputs
    alone (SITL accepts it). A servo left as None is not changed.
    """
    command, params = actuator_params(config, plate_us, clamp_us)
    request = CommandLong.Request()
    request.broadcast = False
    request.command = command
    request.confirmation = 0
    (request.param1, request.param2, request.param3, request.param4,
     request.param5, request.param6, request.param7) = params
    return request


class PayloadWriter:
    """Keeps PX4's payload outputs at the most recently requested positions.

    An addressed command makes MAVROS wait for PX4's reply (about 10 ms over
    serial), and MAVROS refuses a second command of the same type meanwhile.
    So only one command is ever in flight: newer positions wait for the reply,
    and whatever is wanted *then* is sent, skipping positions that went stale
    in between. A command PX4 did not confirm is re-sent after RETRY_SEC.

    Nothing here blocks. Call set() whenever the wanted positions change and
    flush() periodically, which is what retries a failed command.

    For debugging it keeps stats (see summary()) and, with log_commands, logs
    every command sent and every reply.
    """

    def __init__(self, config, client, logger, clock=None, log_commands=False):
        """Args:
            config: PayloadConfig (servo actuator sets and PWM ranges).
            client: rclpy client for mavros_msgs/srv/CommandLong.
            logger: rclpy logger.
            clock: monotonic seconds; defaults to time.monotonic.
            log_commands: log one line per command sent and per reply.
        """
        self.config = config
        self.client = client
        self.logger = logger
        self.clock = clock or time.monotonic
        self.log_commands = log_commands
        self.desired = None     # (plate_us, clamp_us) we want
        self.confirmed = None   # last (plate_us, clamp_us) PX4 confirmed
        self.pending = False    # a command awaits its reply
        self.retry_at = 0.0
        self.failures = 0       # consecutive unconfirmed commands
        self._in_flight = None  # positions of the pending command
        self._wanted_at = 0.0   # when self.desired was set
        self._sent_at = 0.0     # when the pending command went out
        self.reset_stats()

    def reset_stats(self):
        """Start counting afresh, e.g. at the start of each drop."""
        self.sent = 0           # commands sent
        self.accepted = 0       # commands PX4 confirmed
        self.skipped = 0        # wanted positions replaced before being sent
        self.max_reply_s = 0.0  # slowest PX4 reply
        self.max_late_s = 0.0   # longest wait between wanting and sending

    def summary(self):
        """One line of stats since the last reset_stats()."""
        return (f"sent {self.sent}, confirmed {self.accepted}, "
                f"skipped {self.skipped}, slowest reply "
                f"{self.max_reply_s * 1000:.0f} ms, latest send "
                f"{self.max_late_s * 1000:.0f} ms after wanted")

    @property
    def settled(self):
        """True once PX4 has confirmed the positions currently wanted."""
        return not self.pending and self.desired == self.confirmed

    def set(self, plate_us, clamp_us):
        """Want these pulses (us) on the plate and clamp."""
        wanted = (plate_us, clamp_us)
        if wanted != self.desired:
            # The previous wanted positions never went out: they waited for
            # a reply and are now stale. Each one is a skipped pulse.
            if self.desired not in (None, self._in_flight, self.confirmed):
                self.skipped += 1
            self.desired = wanted
            self._wanted_at = self.clock()
        self.flush()

    def flush(self):
        """Send the wanted positions if PX4 has not confirmed them yet."""
        desired = self.desired
        if (desired is None or self.pending or desired == self.confirmed
                or self.clock() < self.retry_at):
            return
        now = self.clock()
        self.pending = True
        self._in_flight = desired
        self._sent_at = now
        self.sent += 1
        late_s = now - self._wanted_at
        self.max_late_s = max(self.max_late_s, late_s)
        if self.log_commands:
            self.logger.info(
                f"send #{self.sent}: plate={desired[0]:g} clamp={desired[1]:g} "
                f"({late_s * 1000:.0f} ms after wanted)")
        future = self.client.call_async(
            actuator_request(self.config, *desired))
        future.add_done_callback(
            lambda done, sent=desired: self._on_reply(done, sent))

    def _on_reply(self, future, sent):
        """Record PX4's verdict on one command. Never raises."""
        self.pending = False
        self._in_flight = None
        reply_s = self.clock() - self._sent_at
        self.max_reply_s = max(self.max_reply_s, reply_s)
        try:
            response = future.result()
            ok = bool(response.success)
            detail = f"result={response.result}"
        except Exception as exc:  # noqa: BLE001 - a callback must not raise
            ok, detail = False, repr(exc)
        if self.log_commands:
            self.logger.info(
                f"{'confirmed' if ok else 'NOT confirmed'} #{self.sent} after "
                f"{reply_s * 1000:.0f} ms" + ('' if ok else f" ({detail})"))
        if ok:
            self.confirmed = sent
            self.failures = 0
            self.accepted += 1
        else:
            self.failures += 1
            self.retry_at = self.clock() + RETRY_SEC
            if self.failures == 1 or self.failures % 10 == 0:
                self.logger.warn(
                    f"Payload servo command not confirmed ({detail}); "
                    f"retrying (failures={self.failures})")
        self.flush()
