from . import log_parse as lp

import beavs_sim as sim


def test_up():
    s = sim.Sim()

    while s.micros < 40000000:
        # This may not be the correct equation
        s.board.bmp.altitude = (((s.micros - 10000000) / 1000000) ** 2) / 2
        if 10000000 < s.micros:
            s.board.bno.acc_x = 30

        s.step()

    content = s.board.serial.contents()
    parsed = [
        lp.parse_serial_line(line) for line in content.split(b"\r\n") if len(line) > 0
    ]

    print([line.altitude for line in parsed if isinstance(line, lp.SerialTelemetry)])
