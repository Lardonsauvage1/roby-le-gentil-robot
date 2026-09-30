"""Protocole serie de la carte B-G431B-ESC1 (firmware SimpleFOC de l'axe 5).

Toutes les positions sont en radians de l'AXE DU BRAS (sortie du reducteur
planetaire 9:1) : la carte fait la conversion x9 en interne.

Pi -> carte (une commande par ligne, terminee par '\\n') :
    P<angle>  consigne de position absolue
    Z<angle>  recalage "l'axe est actuellement a <angle>" (aucun mouvement)
    S         stop (disable moteur)
    E         enable moteur
    R         reset apres defaut
    ?         demande la position

Carte -> Pi :
    "S <pos> <courant> <defaut>"   en continu (~50 Hz)
    "RECALE <val>", "POS <val>", "ENABLED", "DISABLED", "RESET",
    "FAULT STALL", "READY"         reponses ponctuelles

Module sans dependance (ni pyserial ni ROS) : teste unitairement.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Union

# Nombre de decimales envoyees. La carte les lit avec atof() dans un buffer de
# 31 caracteres : 5 decimales = 1e-5 rad bras, largement sous la resolution
# AS5600 ramenee au bras (2*pi / 4096 / 9 = 1.7e-4 rad).
COMMAND_DECIMALS = 5

# Au-dela, atof() de la carte tronquerait la ligne (cmdBuf[32]).
_MAX_COMMAND_LEN = 31


class ProtocolError(ValueError):
    """Commande impossible a encoder de facon sure."""


class EventKind(Enum):
    """Reponses ponctuelles de la carte."""

    RECALE = "RECALE"
    POS = "POS"
    ENABLED = "ENABLED"
    DISABLED = "DISABLED"
    RESET = "RESET"
    FAULT_STALL = "FAULT STALL"
    READY = "READY"


@dataclass(frozen=True)
class StatusFrame:
    """Trame periodique "S <pos> <courant> <defaut>"."""

    position: float
    current: float
    fault: bool


@dataclass(frozen=True)
class Event:
    """Reponse ponctuelle ; value n'est renseignee que pour RECALE et POS."""

    kind: EventKind
    value: Optional[float] = None


Message = Union[StatusFrame, Event]


def _format_angle(angle_rad: float) -> str:
    if not math.isfinite(angle_rad):
        raise ProtocolError(f"angle non fini : {angle_rad!r}")
    # Format fixe : jamais de notation scientifique, que atof() lirait
    # correctement mais qui rallonge la ligne sans raison.
    return f"{angle_rad:.{COMMAND_DECIMALS}f}"


def _encode(text: str) -> bytes:
    if len(text) > _MAX_COMMAND_LEN:
        raise ProtocolError(f"commande trop longue pour la carte : {text!r}")
    return (text + "\n").encode("ascii")


def encode_position(angle_rad: float) -> bytes:
    """Commande P : consigne de position absolue (rad bras)."""
    return _encode("P" + _format_angle(angle_rad))


def encode_recalibrate(angle_rad: float) -> bytes:
    """Commande Z : recalage sans mouvement (rad bras)."""
    return _encode("Z" + _format_angle(angle_rad))


def encode_stop() -> bytes:
    return _encode("S")


def encode_enable() -> bytes:
    return _encode("E")


def encode_reset() -> bytes:
    return _encode("R")


def encode_query() -> bytes:
    return _encode("?")


def _parse_float(token: str) -> Optional[float]:
    try:
        value = float(token)
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def parse_line(line: str) -> Optional[Message]:
    """Decode une ligne recue. Renvoie None si la ligne est inconnue ou abimee.

    Une ligne abimee (octets perdus, debut de trame tronque a l'ouverture du
    port) est ignoree plutot que devinee : la trame suivante arrive 20 ms plus
    tard.
    """
    text = line.strip()
    if not text:
        return None

    parts = text.split()
    head = parts[0]

    if head == "S":
        if len(parts) != 4:
            return None
        pos = _parse_float(parts[1])
        cur = _parse_float(parts[2])
        if pos is None or cur is None or parts[3] not in ("0", "1"):
            return None
        return StatusFrame(position=pos, current=cur, fault=parts[3] == "1")

    if head in ("RECALE", "POS"):
        if len(parts) != 2:
            return None
        value = _parse_float(parts[1])
        if value is None:
            return None
        return Event(EventKind(head), value)

    if text == "FAULT STALL":
        return Event(EventKind.FAULT_STALL)

    if len(parts) == 1 and head in ("ENABLED", "DISABLED", "RESET", "READY"):
        return Event(EventKind(head))

    return None
