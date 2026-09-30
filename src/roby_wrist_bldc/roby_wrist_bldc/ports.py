"""Detection du port serie de la carte B-G431B-ESC1.

La carte est vue a travers son ST-LINK/V2-1 embarque (port COM virtuel,
VID STMicroelectronics 0x0483) sous /dev/ttyACM*. Ordre de recherche :

1. port explicite (tout sauf "auto") ;
2. lien udev stable /dev/roby_wrist (voir udev/99-roby-wrist.rules) ;
3. unique port de VID 0x0483 ;
4. unique /dev/ttyACM* restant, hors peripheriques connus d'autres organes.

On ne choisit JAMAIS entre plusieurs candidats : un faux port ecrit des
consignes "P..." dans un autre organe. Le bras guide (pont USB QinHeng CH343,
VID 0x1a86) est lui aussi en /dev/ttyACM*. Le driver verifie de toute facon
que le port choisi emet des trames "S ..." avant de s'en servir.
"""

from __future__ import annotations

import glob
import os
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence

UDEV_LINK = "/dev/roby_wrist"
ST_VID = 0x0483
# Peripheriques serie connus du robot qui ne sont PAS la carte du poignet.
KNOWN_OTHER_VIDS = {
    0x1A86: "QinHeng CH343 (bras guide STS3215)",
}


class PortNotFoundError(RuntimeError):
    """Aucun port, ou plusieurs candidats indiscernables."""


@dataclass(frozen=True)
class PortInfo:
    device: str
    vid: Optional[int]
    description: str = ""


def _list_ports() -> List[PortInfo]:
    try:
        from serial.tools import list_ports
    except ImportError:  # pyserial absent : on se rabat sur le glob seul
        return [PortInfo(dev, None) for dev in sorted(glob.glob("/dev/ttyACM*"))]
    return [
        PortInfo(p.device, p.vid, p.description or "") for p in list_ports.comports()
    ]


def find_port(
    preferred: str = "auto",
    lister: Callable[[], Sequence[PortInfo]] = _list_ports,
    exists: Callable[[str], bool] = os.path.exists,
) -> str:
    """Renvoie le chemin du port de la carte, ou leve PortNotFoundError."""
    if preferred and preferred != "auto":
        return preferred

    if exists(UDEV_LINK):
        return UDEV_LINK

    ports = list(lister())
    st_ports = [p for p in ports if p.vid == ST_VID]
    if len(st_ports) == 1:
        return st_ports[0].device
    if len(st_ports) > 1:
        raise PortNotFoundError(
            "plusieurs ST-LINK branches, preciser le port ou installer la regle udev : "
            + ", ".join(p.device for p in st_ports)
        )

    acm = [
        p
        for p in ports
        if p.device.startswith("/dev/ttyACM") and p.vid not in KNOWN_OTHER_VIDS
    ]
    if len(acm) == 1:
        return acm[0].device

    listing = ", ".join(
        f"{p.device} ({KNOWN_OTHER_VIDS.get(p.vid or -1, p.description or 'inconnu')})"
        for p in ports
    )
    if not acm:
        raise PortNotFoundError(
            f"carte du poignet introuvable. Ports vus : {listing or 'aucun'}"
        )
    raise PortNotFoundError(
        f"plusieurs /dev/ttyACM* possibles, preciser le port : {listing}"
    )
