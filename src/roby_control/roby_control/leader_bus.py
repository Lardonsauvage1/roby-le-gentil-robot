"""Couche bus du bras guide (Feetech STS3215) — sans ROS, donc testable seule.

Separee de `leader_node` a dessein : les conversions et l'acces au bus se testent
sans rclpy ni materiel (mode `simulate`).

Regle du chantier : ce module est le SEUL a parler au bus serie du leader. Le projet
a deja paye cher d'avoir deux maitres sur un meme bus (contention I2C sur le PCA9685,
servos muets et diagnostic long) — d'ou le verrou exclusif pose sur le port.
"""

from __future__ import annotations

import fcntl
import math
import os
import time

RESOLUTION = 4096  # pas par tour (STS3215)
TWO_PI = 2.0 * math.pi

# Table de registres STS/SMS. Source : lerobot/motors/feetech/tables.py
# (sts3215 -> STS_SMS_SERIES_CONTROL_TABLE). NE PAS confondre avec la serie SCS,
# ou l'adresse 48 est le verrou EEPROM et non la limite de couple.
ADDR_MODEL_NUMBER = (3, 2)
ADDR_TORQUE_ENABLE = (40, 1)
ADDR_GOAL_POSITION = (42, 2)
ADDR_TORQUE_LIMIT = (48, 2)  # RAM : modifiable a chaud, pas besoin d'ouvrir l'EEPROM
ADDR_PRESENT_POSITION = (56, 2)
ADDR_PRESENT_LOAD = (60, 2)
ADDR_PRESENT_VOLTAGE = (62, 1)
ADDR_PRESENT_TEMPERATURE = (63, 1)

MODEL_STS3215 = 777
TORQUE_LIMIT_MAX = 1000  # pleine echelle du registre

# Plages de fonctionnement sain (unites brutes des registres).
VOLT_OK = (60, 84)  # 6,0 .. 8,4 V (dixiemes de volt)
TEMP_MAX = 55  # degres Celsius


def steps_to_rad(steps: int) -> float:
    """Position brute -> radians dans [0, 2pi).

    Convention du LEADER, volontairement brute : aucun deroulement, aucun recentrage.
    La mise en correspondance avec Roby (offsets, sens, bouclage) est l'affaire d'US-019
    et s'appuie sur les bornes de ~/roby_leader_calib.yaml.
    """
    return (float(steps) % RESOLUTION) * TWO_PI / RESOLUTION


def rad_to_steps(rad: float) -> int:
    """Radians -> position brute, ramenee dans [0, 4095]."""
    return int(round((rad % TWO_PI) * RESOLUTION / TWO_PI)) % RESOLUTION


class LeaderBusError(Exception):
    """Erreur typee du bus : jamais une position fausse renvoyee silencieusement."""


class PortUnavailable(LeaderBusError):
    pass


class PortAlreadyOwned(PortUnavailable):
    """Un autre processus detient deja le bus (verrou exclusif)."""


class ServoNotResponding(LeaderBusError):
    pass


class LeaderBus:
    """Acces exclusif au bus serie du bras guide.

    simulate=True : aucun materiel requis, positions synthetiques. Sert au
    developpement, aux tests d'integration et a la mise au point de la stack.
    """

    def __init__(self, port="/dev/roby_leader", baud=1_000_000, ids=(1, 2, 3, 4, 5, 6),
                 simulate=False):
        self.port_name = port
        self.baud = baud
        self.ids = list(ids)
        self.simulate = simulate
        self._port = None
        self._ph = None
        self._sync = None
        self._lock_fd = None
        self._t0 = time.monotonic()
        self._sim_torque = {i: False for i in self.ids}

    # ---------------------------------------------------------------- cycle de vie
    def open(self):
        if self.simulate:
            return
        from scservo_sdk import COMM_SUCCESS, GroupSyncRead, PacketHandler, PortHandler

        self._COMM_SUCCESS = COMM_SUCCESS
        if not os.path.exists(self.port_name):
            raise PortUnavailable("port introuvable : %s" % self.port_name)

        # Verrou exclusif : deux instances du noeud ne doivent JAMAIS partager le bus.
        # Un /dev/ttyACM* peut etre ouvert deux fois sous Linux, d'ou le flock explicite.
        self._lock_fd = os.open(self.port_name, os.O_RDWR)
        try:
            fcntl.flock(self._lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            os.close(self._lock_fd)
            self._lock_fd = None
            raise PortAlreadyOwned(
                "le bus %s est deja detenu par un autre processus "
                "(une autre instance du noeud ?)" % self.port_name
            )

        self._port = PortHandler(self.port_name)
        if not self._port.openPort():
            self._release_lock()
            raise PortUnavailable("ouverture impossible : %s" % self.port_name)
        if not self._port.setBaudRate(self.baud):
            self._port.closePort()
            self._release_lock()
            raise PortUnavailable("baudrate %d refuse" % self.baud)
        self._ph = PacketHandler(0)  # protocol_end STS/SMS
        addr, size = ADDR_PRESENT_POSITION
        self._sync = GroupSyncRead(self._port, self._ph, addr, size)
        for sid in self.ids:
            self._sync.addParam(sid)

    def close(self):
        if self.simulate:
            return
        if self._port is not None:
            self._port.closePort()
            self._port = None
        self._release_lock()

    def _release_lock(self):
        if self._lock_fd is not None:
            try:
                fcntl.flock(self._lock_fd, fcntl.LOCK_UN)
            finally:
                os.close(self._lock_fd)
                self._lock_fd = None

    # ---------------------------------------------------------------- lectures
    def ping_all(self):
        """Retourne la liste des IDs qui repondent."""
        if self.simulate:
            return list(self.ids)
        return [i for i in self.ids if self._ph.ping(self._port, i)[1] == self._COMM_SUCCESS]

    def read_positions(self):
        """Lecture SYNCHRONE : une transaction pour tous les servos.

        Retourne {id: pas}. Leve ServoNotResponding si la transaction echoue ou si un
        servo manque : on ne renvoie JAMAIS une position partielle ou perimee.
        """
        if self.simulate:
            t = time.monotonic() - self._t0
            return {
                sid: int((RESOLUTION / 2) + (RESOLUTION / 4) * math.sin(0.2 * t + k))
                % RESOLUTION
                for k, sid in enumerate(self.ids)
            }
        if self._sync is None:
            raise PortUnavailable("bus non ouvert")
        if self._sync.txRxPacket() != self._COMM_SUCCESS:
            raise ServoNotResponding("lecture synchrone en echec (bus muet ?)")
        addr, size = ADDR_PRESENT_POSITION
        out, absents = {}, []
        for sid in self.ids:
            if self._sync.isAvailable(sid, addr, size):
                out[sid] = self._sync.getData(sid, addr, size)
            else:
                absents.append(sid)
        if absents:
            raise ServoNotResponding("aucune reponse des servos %s" % absents)
        return out

    def _read_reg(self, sid, reg):
        addr, size = reg
        if size == 1:
            v, comm, err = self._ph.read1ByteTxRx(self._port, sid, addr)
        else:
            v, comm, err = self._ph.read2ByteTxRx(self._port, sid, addr)
        return None if (comm != self._COMM_SUCCESS or err != 0) else v

    def read_telemetry(self):
        """{id: {volt_dV, temp_C, load, torque_on}} — None si un champ est illisible."""
        if self.simulate:
            return {sid: {"volt_dV": 74, "temp_C": 30, "load": 0,
                          "torque_on": self._sim_torque[sid]} for sid in self.ids}
        out = {}
        for sid in self.ids:
            raw_load = self._read_reg(sid, ADDR_PRESENT_LOAD)
            out[sid] = {
                "volt_dV": self._read_reg(sid, ADDR_PRESENT_VOLTAGE),
                "temp_C": self._read_reg(sid, ADDR_PRESENT_TEMPERATURE),
                "load": decode_load(raw_load),
                "torque_on": bool(self._read_reg(sid, ADDR_TORQUE_ENABLE)),
            }
        return out

    # ---------------------------------------------------------------- ecritures
    def _write_reg(self, sid, reg, value):
        addr, size = reg
        if size == 1:
            comm, err = self._ph.write1ByteTxRx(self._port, sid, addr, value)
        else:
            comm, err = self._ph.write2ByteTxRx(self._port, sid, addr, value)
        return comm == self._COMM_SUCCESS and err == 0

    def set_torque(self, enable, ids=None):
        """Active/coupe le couple. ids=None => tous. Retourne la liste des echecs.

        ATTENTION SECURITE — a l'ACTIVATION, on ecrit d'abord `Goal_Position` = position
        mesuree. Sans cela, le servo qui reprend son couple se precipite vers la derniere
        cible qu'il a en memoire : une valeur qui peut dater d'un usage anterieur, voire
        d'usine. Sur un bras que l'operateur tient en main, c'est un saut brutal vers une
        position arbitraire. On fige donc la cible sur l'endroit ou le bras se trouve
        DEJA : activer le couple ne doit produire AUCUN mouvement.
        """
        targets = list(self.ids if ids is None else ids)
        if self.simulate:
            for sid in targets:
                self._sim_torque[sid] = bool(enable)
            return []
        echecs = []
        if enable:
            positions = self.read_positions()  # leve si le bus est muet : on n'active PAS
            for sid in targets:
                if not self._write_reg(sid, ADDR_GOAL_POSITION, positions[sid]):
                    echecs.append(sid)
            if echecs:
                # Cible non figee sur ces servos : ne surtout pas leur donner de couple.
                targets = [s for s in targets if s not in echecs]
        return echecs + [sid for sid in targets
                         if not self._write_reg(sid, ADDR_TORQUE_ENABLE, 1 if enable else 0)]

    def set_goal_position(self, cibles):
        """Ecrit des positions cibles {id: pas}. Reserve a l'assistance gravite (US-025)
        et au realignement (US-022) : sans couple actif, cela n'a aucun effet."""
        if self.simulate:
            return []
        return [sid for sid, pas in cibles.items()
                if not self._write_reg(sid, ADDR_GOAL_POSITION, int(pas) % RESOLUTION)]

    def set_torque_limit(self, value, ids=None):
        """Limite de couple, 0..1000 (registre RAM). Retourne la liste des echecs."""
        if not 0 <= value <= TORQUE_LIMIT_MAX:
            raise ValueError("limite de couple hors plage 0..%d : %s"
                             % (TORQUE_LIMIT_MAX, value))
        targets = list(self.ids if ids is None else ids)
        if self.simulate:
            return []
        return [sid for sid in targets
                if not self._write_reg(sid, ADDR_TORQUE_LIMIT, int(value))]


def decode_load(raw):
    """Charge STS : magnitude 10 bits + bit de signe en position 10."""
    if raw is None:
        return None
    return (-1 if raw & (1 << 10) else 1) * (raw & 0x3FF)


def telemetry_warnings(sid, t):
    """Retourne la liste des anomalies lisibles pour un servo (vide si tout va bien)."""
    warns = []
    v, temp = t.get("volt_dV"), t.get("temp_C")
    if v is None:
        warns.append("ID %d : tension illisible" % sid)
    elif not VOLT_OK[0] <= v <= VOLT_OK[1]:
        warns.append("ID %d : tension %.1f V hors plage [%.1f..%.1f]"
                     % (sid, v / 10.0, VOLT_OK[0] / 10.0, VOLT_OK[1] / 10.0))
    if temp is None:
        warns.append("ID %d : temperature illisible" % sid)
    elif temp >= TEMP_MAX:
        warns.append("ID %d : temperature %d C >= %d C" % (sid, temp, TEMP_MAX))
    return warns
