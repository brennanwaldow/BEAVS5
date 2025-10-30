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
class LogApogeeAltitude(Log):
    max_height_m: float
    max_height_ft: float


# TODO: Make enum
@dataclass
class LogMessage(Log):
    message: str


@dataclass
class SerialLog(SerialLine):
    time: int
    log: Log


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

    apogee_res = parse(
        "Altitude achieved: {max_height_m:g} m AGL    //    {max_height_ft:g} ft AGL",
        text,
    )
    if apogee_res is not None:
        return LogApogeeAltitude(
            max_height_m=apogee_res["max_height_m"],
            max_height_ft=apogee_res["max_height_ft"],
        )

    return LogMessage(message=text)


def parse_serial_line(rawline: bytes) -> SerialLine | None:
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
