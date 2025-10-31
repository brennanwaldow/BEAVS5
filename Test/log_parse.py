from enum import Enum
from parse import parse
from dataclasses import dataclass


class SerialLine:
    pass


@dataclass
class SerialInit(SerialLine):
    pass


class InitError(Enum):
    BMP = 0
    BNO = 1
    SD = 2


@dataclass
class SerialInitError(SerialLine):
    error: InitError


class Log:
    pass


class FlightPhase(Enum):
    PREFLIGHT = 0
    DISARMED = 1
    ARMED = 2
    FLIGHT = 3
    COAST = 4
    OVERSHOOT = 5
    DESCENT = 6


@dataclass
class LogPhaseChange(Log):
    phase: FlightPhase


@dataclass
class LogMessage(Log):
    message: str


@dataclass
class SerialLog(SerialLine):
    time: int
    log: Log


@dataclass
class SerialMessage(SerialLine):
    message: str


@dataclass
class SerialTelemetry(SerialLine):
    roll: float
    roll_rel: float
    pitch_x: float
    pitch_y: float
    pitch: float
    altitude: float
    prev_altitude: float
    dt: float
    acceleration: float
    velocity: float


def parse_log_text(text: str) -> Log | None:
    if text == "Safety pin removed. BEAVS arming.":
        return LogPhaseChange(phase=FlightPhase.ARMED)
    if text == "Safety pin removed quickly. Switching to preflight.":
        return LogPhaseChange(phase=FlightPhase.PREFLIGHT)
    if text == "Safety pin inserted. Booting to Disarmed.":
        return LogPhaseChange(phase=FlightPhase.DISARMED)
    if text == "Safety pin reinserted. Disarming.":
        return LogPhaseChange(phase=FlightPhase.DISARMED)
    if text == "Motor ignition detected.":
        return LogPhaseChange(phase=FlightPhase.FLIGHT)
    if text == "Coast phase entered, beginning deployment.":
        return LogPhaseChange(phase=FlightPhase.COAST)
    if text == "Target apogee overshot! Extending full braking.":
        return LogPhaseChange(phase=FlightPhase.OVERSHOOT)
    if text == "Apogee reached.":
        return LogPhaseChange(phase=FlightPhase.DESCENT)

    return LogMessage(message=text)


def parse_serial_line(rawline: bytes) -> SerialLine:
    line = rawline.decode()

    if line == "Starting up hardware...":
        return SerialInit()

    if line == "ERROR: BMP390 failed to initialize.":
        return SerialInitError(error=InitError.BMP)

    if line == "ERROR: BNO055 failed to initialize.":
        return SerialInitError(error=InitError.BNO)

    if line == "ERROR: SD card writer failed to initialize.":
        return SerialInitError(error=InitError.SD)

    res = parse("[{time:g}s] {text}", line)
    if res is not None:
        return SerialLog(res["time"], parse_log_text(res["text"]))

    res = parse(
        "{roll:g} {roll_rel:g} {pitch_x:g} {pitch_y:g} {pitch:g} {altitude:g} {prev_altitude:g} {dt:g} {acceleration:g} {velocity:g}",
        line,
    )
    if res is not None:
        return SerialTelemetry(
            res["roll"],
            res["roll_rel"],
            res["pitch_y"],
            res["pitch_x"],
            res["pitch"],
            res["altitude"],
            res["prev_altitude"],
            res["dt"],
            res["acceleration"],
            res["velocity"],
        )

    print(len(line.split(" ")))
    return SerialMessage(line)
