if __name__ == '__main__':
    import atexit
    from collections import Counter
    from contextlib import nullcontext
    import sys
    import time
    from unittest.mock import patch

    from amr_ros2_bridge_v3 import Uart
    from amr_ros2_bridge_v3_2 import Limits, run_ros
    from amr_ros2_bridge_v3_3 import BridgeV33
    from test_amr_uart_startup import Firmware33
    from tools.amr_cmd_vel_adapter import run_ros as run_adapter

    if sys.argv[1] == 'bridge':
        class RealClock:
            now = staticmethod(time.monotonic)
            wall = staticmethod(time.time_ns)

        class TimedFirmware(Firmware33):
            updated_at = None
            encoders = [0., 0.]

            def reply(self, line):
                now = time.monotonic()
                if self.updated_at is not None and self.running:
                    for i, rpm in enumerate(self.speed):
                        self.encoders[i] += rpm * 3960 / 60 * (now - self.updated_at)
                self.updated_at = now
                self.ticks = tuple(map(round, self.encoders))
                if line == 'Q':
                    time.sleep(.030)
                return super().reply(line)

        firmware = TimedFirmware(RealClock())
        def report_uart():
            print('UART COUNTS', dict(Counter(cmd.split()[0] for cmd in firmware.log)), flush=True)
        atexit.register(report_uart)
        with patch('amr_ros2_bridge_v3_2.exclusive_uart', return_value=nullcontext(Uart(firmware))):
            sys.exit(run_ros('fake', 'ground', 'nav2', 30, Limits(), True, bridge_class=BridgeV33))

    if sys.argv[1] == 'adapter':
        sys.exit(run_adapter(sys.argv[2], 'nav2', True, stationary_check=sys.argv[3] == "idle", laptop='127.0.0.1'))
