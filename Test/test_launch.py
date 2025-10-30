from . import log_parse as lp

import beavs_sim as sim


def test_up():
    print("asdfasf")
    s = sim.Sim()

    s.step_to(10000000)

    content = s.board.serial.contents()
    parsed = [
        lp.parse_serial_line(line) for line in content.split(b"\r\n") if len(line) > 0
    ]

    print(parsed)
