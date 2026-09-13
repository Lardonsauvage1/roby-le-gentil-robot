import math

import pytest

from roby_wrist_bldc import protocol
from roby_wrist_bldc.protocol import Event, EventKind, StatusFrame


def test_encode_commands():
    assert protocol.encode_position(1.57) == b"P1.57000\n"
    assert protocol.encode_position(-0.2) == b"P-0.20000\n"
    assert protocol.encode_recalibrate(0.2009) == b"Z0.20090\n"
    assert protocol.encode_stop() == b"S\n"
    assert protocol.encode_enable() == b"E\n"
    assert protocol.encode_reset() == b"R\n"
    assert protocol.encode_query() == b"?\n"


def test_encode_never_uses_scientific_notation():
    assert protocol.encode_position(1e-7) == b"P0.00000\n"
    assert b"e" not in protocol.encode_position(-3e-9)


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_encode_rejects_non_finite(bad):
    with pytest.raises(protocol.ProtocolError):
        protocol.encode_position(bad)


def test_encode_rejects_line_too_long_for_board_buffer():
    with pytest.raises(protocol.ProtocolError):
        protocol.encode_position(1e25)


@pytest.mark.parametrize(
    "line, expected",
    [
        ("S 0.2009 0.12 0\r\n", StatusFrame(0.2009, 0.12, False)),
        ("S -1.5000 2.71 1", StatusFrame(-1.5, 2.71, True)),
        ("RECALE 0.2009\r\n", Event(EventKind.RECALE, 0.2009)),
        ("POS -0.1000", Event(EventKind.POS, -0.1)),
        ("ENABLED\r\n", Event(EventKind.ENABLED)),
        ("DISABLED", Event(EventKind.DISABLED)),
        ("RESET", Event(EventKind.RESET)),
        ("FAULT STALL\r\n", Event(EventKind.FAULT_STALL)),
        ("READY", Event(EventKind.READY)),
    ],
)
def test_parse_valid_lines(line, expected):
    assert protocol.parse_line(line) == expected


@pytest.mark.parametrize(
    "line",
    [
        "",
        "\r\n",
        "S 0.20",  # trame tronquee (ouverture du port en cours de trame)
        "S 0.2009 0.12",
        "S 0.2009 0.12 2",
        "S abc 0.12 0",
        "S nan 0.12 0",
        ".2009 0.12 0",
        "RECALE",
        "RECALE x",
        "POS 1 2",
        "FAULT",
        "READY now",
        "MOT: Align sensor.",  # message de debug SimpleFOC eventuel
    ],
)
def test_parse_rejects_garbage(line):
    assert protocol.parse_line(line) is None
