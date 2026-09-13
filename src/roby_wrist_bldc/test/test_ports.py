import pytest

from roby_wrist_bldc.ports import UDEV_LINK, PortInfo, PortNotFoundError, find_port

ST = 0x0483
CH343 = 0x1A86


def finder(ports, udev=False):
    return lambda preferred="auto": find_port(
        preferred, lister=lambda: ports, exists=lambda path: udev and path == UDEV_LINK
    )


def test_explicit_port_is_used_as_is():
    assert finder([])("/dev/ttyACM3") == "/dev/ttyACM3"


def test_udev_link_wins():
    assert finder([PortInfo("/dev/ttyACM0", ST)], udev=True)() == UDEV_LINK


def test_single_stlink():
    ports = [PortInfo("/dev/ttyACM0", CH343), PortInfo("/dev/ttyACM1", ST)]
    assert finder(ports)() == "/dev/ttyACM1"


def test_two_stlinks_is_ambiguous():
    with pytest.raises(PortNotFoundError):
        finder([PortInfo("/dev/ttyACM0", ST), PortInfo("/dev/ttyACM1", ST)])()


def test_leader_arm_bridge_is_never_picked():
    with pytest.raises(PortNotFoundError, match="introuvable"):
        finder([PortInfo("/dev/ttyACM0", CH343)])()


def test_single_unknown_acm_fallback():
    ports = [
        PortInfo("/dev/ttyACM0", CH343),
        PortInfo("/dev/ttyACM1", None),
        PortInfo("/dev/ttyAMA0", None),
    ]
    assert finder(ports)() == "/dev/ttyACM1"


def test_several_unknown_acm_is_ambiguous():
    with pytest.raises(PortNotFoundError, match="plusieurs"):
        finder([PortInfo("/dev/ttyACM0", None), PortInfo("/dev/ttyACM1", None)])()


def test_nothing_plugged():
    with pytest.raises(PortNotFoundError, match="aucun"):
        finder([])()
