"""leader_node — publie l'etat du bras guide et expose le controle de son couple (US-018).

Ce noeud est le SEUL proprietaire du bus serie du leader. Tout le reste de la stack
passe par ses topics et ses services : le projet a deja paye cher d'avoir deux maitres
sur un meme bus (contention I2C sur le PCA9685, servos muets, diagnostic long).
Un verrou exclusif sur le port fait respecter la regle plutot que de l'esperer.

Le noeud demarre COUPLE COUPE et coupe le couple a l'arret, y compris sur Ctrl-C :
le bras guide se manipule a la main, l'activation du couple est un acte explicite.

Aucune commande n'est envoyee vers le vrai bras Roby : ce noeud ne fait que lire le
leader et gerer son couple.

Lancement (ROS 2 et LeRobot doivent coexister dans le meme interpreteur) :
    bash ~/roby_leader_node.sh
    bash ~/roby_leader_node.sh --sim        # aucun materiel requis
"""

from __future__ import annotations

import functools
import math
import time
import os
import subprocess
import xml.etree.ElementTree as ET

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

from roby_control.leader_joystick import (RESOLUTION, cible_de_rappel, commande_axe,
                                          pas_court)
from roby_control.leader_mapping import charger as charger_calib
from roby_control.leader_bus import (
    LeaderBus,
    rad_to_steps,
    LeaderBusError,
    PortAlreadyOwned,
    PortUnavailable,
    TORQUE_LIMIT_MAX,
    steps_to_rad,
    telemetry_warnings,
)

# Noms du LEADER, pas ceux de Roby : ce topic porte des angles bruts du bras guide.
# Les noms URDF (joint_1...) n'apparaissent qu'APRES conversion (US-019), sinon on
# croit lire un angle de Roby alors qu'il n'a ete ni signe, ni decale, ni mis a l'echelle.
DEFAULT_JOINTS = ["axe_1_base", "axe_2_epaule", "axe_3_coude",
                  "axe_4_poignet", "axe_5_rot_poignet", "pince"]
DEFAULT_IDS = [1, 2, 3, 4, 5, 6]
FAILS_BEFORE_STALE = 5  # lectures ratees consecutives avant de declarer le bus perdu



def verifier_interface_dds():
    """L'interface Ethernet declaree dans la config DDS doit etre operationnelle.

    ERREUR BLOQUANTE si elle ne l'est pas. Aucun repli n'est propose — ni Wi-Fi, ni
    boucle locale : un repli silencieux donnerait une stack qui a l'air de fonctionner
    alors qu'elle ne parle pas au Pi5. Le cablage se corrige, il ne se contourne pas.

    Retourne le nom de l'interface validee.
    """
    uri = os.environ.get("CYCLONEDDS_URI", "")
    if not uri:
        raise SystemExit(
            "ERREUR BLOQUANTE : CYCLONEDDS_URI n'est pas defini.\n"
            "Le noeud doit tourner sur le reseau du projet. Lancer via "
            "~/roby_leader_node.sh, qui charge ~/cyclone_config.xml."
        )
    chemin = uri.replace("file://", "")
    if not os.path.exists(chemin):
        raise SystemExit("ERREUR BLOQUANTE : config DDS introuvable : %s" % chemin)

    try:
        racine = ET.parse(chemin).getroot()
    except ET.ParseError as e:
        raise SystemExit("ERREUR BLOQUANTE : config DDS illisible (%s) : %s" % (chemin, e))

    noms = [el.get("name") for el in racine.iter()
            if el.tag.endswith("NetworkInterface") and el.get("name")]
    if not noms:
        raise SystemExit(
            "ERREUR BLOQUANTE : aucune <NetworkInterface> declaree dans %s.\n"
            "L'interface du lien PC <-> Pi5 doit y etre nommee explicitement." % chemin
        )

    for nom in noms:
        base = "/sys/class/net/%s" % nom
        if not os.path.isdir(base):
            raise SystemExit(
                "ERREUR BLOQUANTE : l'interface '%s' declaree dans %s n'existe pas sur "
                "cette machine.\nInterfaces disponibles : %s\n"
                "=> Corriger la config DDS, ou brancher la bonne carte."
                % (nom, chemin, ", ".join(sorted(os.listdir("/sys/class/net"))))
            )
        try:
            with open(base + "/operstate") as f:
                etat = f.read().strip()
            with open(base + "/carrier") as f:
                porteuse = f.read().strip()
        except OSError:
            etat, porteuse = "inconnu", "0"
        if etat != "up" or porteuse != "1":
            raise SystemExit(
                "ERREUR BLOQUANTE : l'interface Ethernet '%s' n'est pas operationnelle "
                "(operstate=%s, carrier=%s).\n"
                "Le bras guide doit tourner sur le reseau CABLE du projet, jamais en "
                "Wi-Fi ni en boucle locale.\n"
                "=> BRANCHER le cable Ethernet sur '%s' et verifier qu'il obtient une "
                "adresse en 192.168.2.x, puis relancer.\n"
                "Ce n'est pas a contourner : un repli silencieux donnerait une stack qui "
                "semble marcher sans parler au Pi5." % (nom, etat, porteuse, nom)
            )
        adresses = [
            ligne for ligne in
            subprocess.run(["ip", "-4", "-brief", "addr", "show", nom],
                           capture_output=True, text=True).stdout.split()
            if "." in ligne
        ]
        if not adresses:
            raise SystemExit(
                "ERREUR BLOQUANTE : l'interface '%s' est active mais n'a AUCUNE adresse "
                "IPv4.\n=> Verifier la configuration reseau (192.168.2.x attendu)." % nom
            )
    return noms[0]


class LeaderNode(Node):
    def __init__(self):
        super().__init__("leader_node")

        self.declare_parameter("port", "/dev/roby_leader")
        self.declare_parameter("baud", 1_000_000)
        self.declare_parameter("ids", DEFAULT_IDS)
        self.declare_parameter("joint_names", DEFAULT_JOINTS)
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("telemetry_rate_hz", 1.0)
        self.declare_parameter("simulate", False)
        self.declare_parameter("torque_limit", TORQUE_LIMIT_MAX)
        # --- mode joystick (rappel elastique vers le zero + consigne de vitesse) ---
        self.declare_parameter("calib_file", "")
        # Zone morte PAR AXE, meme convention que joystick_torque_max_pct :
        # une seule valeur s'applique a tous, sinon autant de valeurs que d'axes.
        # Les axes qui portent le bras (base, epaule, coude) demandent une zone
        # plus large : la main y exerce des micro-efforts en permanence, et une
        # zone trop etroite fait deriver le robot a l'arret.
        self.declare_parameter("joystick_deadzone_deg", [5.0])
        # Couple max par AXE (aligne sur `ids`). Une seule valeur = la meme partout.
        # Les axes porteurs (epaule, coude) en demandent plus : ramener au neutre exige
        # de SOULEVER le bras, bien plus que de le maintenir.
        self.declare_parameter("joystick_torque_max_pct", [10.0])
        # Couple DES LA SORTIE de la zone morte (0 = demarrage progressif depuis zero).
        # Sans plancher, il existe une bande ou le rappel est actif mais trop faible pour
        # vaincre le frottement : le bras y stagne.
        self.declare_parameter("joystick_torque_min_pct", [0.0])
        # Ecart maximal demande au servo en une fois (pas).
        # /!\ Ce n'est PAS qu'un garde-fou : le servo produit un couple proportionnel a
        # son ERREUR de position, donc cette borne plafonne aussi la force du rappel.
        # Trop petite, elle bride le couple bien avant `Torque_Limit` — vecu le
        # 2026-09-05 : l'epaule ne demandait que 46 de charge avec une limite a 250.
        # La valeur etait basse pour empecher de franchir le bouclage ; depuis le
        # recentrage des codeurs, plus aucun axe ne le franchit.
        self.declare_parameter("joystick_pas_max", 800)
        self.declare_parameter("joystick_rate_hz", 20.0)
        # Garde anti-emballement : au-dela de cette vitesse mesuree, on coupe tout.
        self.declare_parameter("joystick_vitesse_max_dps", 300.0)
        # --- recentrage motorise (US-022) : le guide rejoint sa pose de reference ---
        # Couple volontairement bas : le bras doit pouvoir etre retenu a la main sans
        # effort. Ce n'est pas un actionneur de puissance, c'est une aide au placement.
        self.declare_parameter("recentrage_couple_pct", 25.0)
        self.declare_parameter("recentrage_pas_par_cycle", 60)
        self.declare_parameter("recentrage_tolerance_deg", 2.0)
        self.declare_parameter("recentrage_timeout_s", 12.0)
        # Couple CONSERVE apres le recentrage, pour que le guide tienne sa pose au lieu
        # de s'affaisser des qu'on le lache. 0 = on recoupe (bras libre a la main).
        # Ce n'est pas la meme grandeur que `recentrage_couple_pct` : bouger le bras
        # demande de vaincre les frottements, le maintenir demande seulement de porter
        # son poids -- en general moins.
        self.declare_parameter("recentrage_maintien_pct", 0.0)
        # MAINTIEN (assistance gravite, US-025) : le guide reste ou on le laisse, au
        # lieu de retomber. A ne pas confondre avec le mode joystick, qui RAMENE vers
        # le zero -- utile en pilotage vitesse, nefaste en pilotage position ou le
        # retour au centre est traduit en mouvement du grand bras.
        # La cible est reecrite en continu sur la position courante : le guide suit donc
        # la main sans resister, et tient des qu'on le lache.
        self.declare_parameter("maintien_couple_pct", 30.0)
        self.declare_parameter("maintien_rate_hz", 20.0)
        # BANDE MORTE du maintien, en degres. Sans elle, reecrire la cible sur la
        # position courante a chaque cycle donne un servo SANS erreur, donc sans couple
        # -- le bras s'affaisse et le cycle suivant enregistre la position affaissee
        # comme nouvelle cible : il suit sa propre chute. Vecu le 2026-09-09.
        # Au-dela du seuil on considere que l'operateur DEPLACE le bras et on suit ;
        # en deca, on tient la derniere cible et le servo resiste a la gravite.
        self.declare_parameter("maintien_bande_morte_deg", 3.0)
        # Seuil de RELACHEMENT, plus bas que la bande morte : une fois que l'operateur
        # a commence a deplacer le bras, la cible le SUIT librement jusqu'a ce qu'il
        # s'arrete. Sans cet hysteresis, il faut vaincre le couple a chaque increment
        # de la bande morte -- le bras parait dur et saccade. Avec, il est ferme a
        # l'arret et souple des qu'on le pousse.
        self.declare_parameter("maintien_relache_deg", 0.8)
        # Duree d'immobilite avant de reprendre la tenue. Court = il tient vite mais
        # se crispe au moindre arret de la main ; long = plus doux, mais il s'affaisse
        # davantage avant d'etre rattrape.
        self.declare_parameter("maintien_delai_s", 0.25)
        # Duree MAXIMALE pendant laquelle le couple peut rester coupe. Sans cette
        # borne, une chute est detectee comme un deplacement volontaire : le couple
        # reste coupe, donc le bras continue de tomber, ce qui entretient la detection.
        # Il ne se rattrape jamais. Au-dela de ce delai on reprend la main d'office ;
        # si l'operateur bougeait vraiment, son geste suivant relachera de nouveau.
        self.declare_parameter("maintien_libre_max_s", 1.5)
        # REPRISE AUTOMATIQUE de la tenue apres une pause. Desactivee par defaut : se
        # redurcir tout seul pendant que la main est encore sur le bras est deroutant,
        # et aucun reglage de delai ne separe vraiment "l'operateur marque une pause"
        # de "l'operateur a lache". Sans reprise, le comportement est previsible : le
        # guide TIENT tant qu'on ne l'a pas touche, et reste LIBRE des qu'on l'a
        # deplace, jusqu'a ce qu'on rearme la tenue (bouton maintien ou recentrage).
        self.declare_parameter("maintien_reprise_auto", False)

        g = self.get_parameter
        self.ids = list(g("ids").value)
        self.joint_names = list(g("joint_names").value)
        # Si l'appelant restreint les IDs sans redonner les noms (cas courant : la pince
        # n'est pas chainee), on derive les noms des IDs plutot que d'exiger la liste.
        if self.joint_names == DEFAULT_JOINTS and len(self.ids) != len(DEFAULT_JOINTS):
            self.joint_names = [
                DEFAULT_JOINTS[i - 1] if 1 <= i <= len(DEFAULT_JOINTS) else "id_%d" % i
                for i in self.ids
            ]
        self.simulate = bool(g("simulate").value)
        if len(self.ids) != len(self.joint_names):
            raise ValueError(
                "ids (%d) et joint_names (%d) doivent avoir la meme longueur"
                % (len(self.ids), len(self.joint_names))
            )
        self.name_of = dict(zip(self.ids, self.joint_names))

        self.bus = LeaderBus(
            port=g("port").value, baud=int(g("baud").value),
            ids=self.ids, simulate=self.simulate,
        )
        try:
            self.bus.open()
        except PortAlreadyOwned as e:
            self.get_logger().fatal(
                "%s\nUne seule instance de leader_node peut detenir le bus. "
                "Arreter l'autre avant de relancer." % e
            )
            raise
        except PortUnavailable as e:
            self.get_logger().fatal(
                "%s\nVerifier que la carte est branchee (/dev/roby_leader, regle udev) "
                "et que l'alimentation 7,4 V debite bien (mode CV, pas CC)." % e
            )
            raise

        if self.simulate:
            self.get_logger().warn(
                "MODE SIMULATION : aucun materiel n'est lu, les positions sont "
                "synthetiques. Ne pas en tirer de conclusion sur le vrai bras."
            )
        else:
            present = self.bus.ping_all()
            manquants = [i for i in self.ids if i not in present]
            if manquants:
                self.get_logger().error(
                    "Servos attendus mais absents du bus : %s (presents : %s). "
                    "Verifier le chainage et l'alimentation." % (manquants, present)
                )
            else:
                self.get_logger().info("Les %d servos repondent : %s"
                                       % (len(present), present))

        # Etat sur : le couple est coupe au demarrage, quel que soit l'etat precedent.
        echecs = self.bus.set_torque(False)
        if echecs:
            self.get_logger().error("Couple NON coupe sur %s au demarrage !" % echecs)
        else:
            self.get_logger().info("Couple coupe sur tous les servos (etat de depart sur).")
        self._apply_torque_limit(int(g("torque_limit").value), initial=True)

        self.pub_js = self.create_publisher(JointState, "/leader/joint_states", 10)
        self.pub_tel = self.create_publisher(DiagnosticArray, "/leader/telemetry", 5)

        # Signal de recentrage, en QoS durable : tout noeud de teleoperation doit
        # pouvoir se figer pendant que le guide rejoint sa pose de reference, sinon il
        # traduit ce trajet en mouvement du grand bras. Durable pour que le noeud qui
        # demarre en retard connaisse quand meme l'etat courant.
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_recentrage = self.create_publisher(Bool, "/leader/recentrage", qos)
        self.pub_recentrage.publish(Bool(data=False))
        self.srv_maintien = self.create_service(
            SetBool, "/leader/maintien", self._sur_bus(self._srv_maintien))
        self.srv_recentrer = self.create_service(
            Trigger, "/leader/recentrer", self._sur_bus(self._srv_recentrer))
        self.srv_torque = self.create_service(
            SetBool, "/leader/set_torque", self._sur_bus(self._srv_set_torque_all)
        )
        self.srv_joint = [
            self.create_service(
                SetBool, "/leader/%s/set_torque" % name,
                self._sur_bus(functools.partial(self._srv_set_torque_one, sid)),
            )
            for sid, name in self.name_of.items()
        ]
        self.add_on_set_parameters_callback(self._on_param)

        rate = max(1.0, float(g("publish_rate_hz").value))
        trate = max(0.1, float(g("telemetry_rate_hz").value))
        self.timer_js = self.create_timer(1.0 / rate, self._tick_positions)
        self.timer_tel = self.create_timer(1.0 / trate, self._tick_telemetry)

        # ---- mode joystick ----
        self.joy_actif = False
        self.maintien = False
        self._maintien_cible = {}
        self._maintien_bouge = {}
        self._maintien_prec = None
        self._maintien_calme = 0
        self._maintien_tient = False
        self._maintien_libre = 0
        self.verrou = False
        self.joy_deadzone = self._deadzones(g("joystick_deadzone_deg").value)
        pcts = list(g("joystick_torque_max_pct").value)
        if len(pcts) == 1:
            pcts = pcts * len(self.ids)
        if len(pcts) != len(self.ids):
            raise ValueError("joystick_torque_max_pct : %d valeurs pour %d axes"
                             % (len(pcts), len(self.ids)))
        self.joy_couple_max = {
            sid: int(TORQUE_LIMIT_MAX * float(p) / 100.0)
            for sid, p in zip(self.ids, pcts)
        }
        self.joy_pas_max = int(g("joystick_pas_max").value)
        mins = list(g("joystick_torque_min_pct").value)
        if len(mins) == 1:
            mins = mins * len(self.ids)
        self.joy_couple_min = {sid: int(TORQUE_LIMIT_MAX * float(m) / 100.0)
                               for sid, m in zip(self.ids, mins)}
        self.joy_vmax = math.radians(float(g("joystick_vitesse_max_dps").value))
        self._joy_prev = None
        self._joy_dernier_couple = {}
        chemin = g("calib_file").value or None
        self._calib_chemin = chemin
        try:
            self.cal = charger_calib(chemin)
            manquants = self.cal.non_calibres()
            if manquants:
                self.get_logger().warn(
                    "Calibration incomplete pour %s : le mode joystick refusera de "
                    "demarrer sur ces axes." % manquants
                )
            else:
                self.get_logger().info(
                    "Calibration chargee : %d articulations." % len(self.cal.joints))
        except Exception as e:  # noqa: BLE001
            self.cal = None
            self.get_logger().warn(
                "Calibration illisible (%s) : mode joystick indisponible." % e)

        self.pub_joy = self.create_publisher(JointState, "/leader/joystick", 10)
        self.srv_joy = self.create_service(
            SetBool, "/leader/joystick", self._sur_bus(self._srv_joystick))
        self.timer_joy = self.create_timer(
            1.0 / max(1.0, float(g("joystick_rate_hz").value)), self._tick_joystick)
        self.create_timer(
            1.0 / max(1.0, float(g("maintien_rate_hz").value)), self._tick_maintien)
        self._mtime_calib_vu = self._mtime_calib()
        self.create_timer(0.5, self._recharger_si_change)

        self._fails = 0
        self._stale = False
        self._dernier_pos = None
        self.get_logger().info(
            "leader_node pret : %d articulations, publication a %.0f Hz, "
            "telemetrie a %.1f Hz." % (len(self.ids), rate, trate)
        )

    # ------------------------------------------------------------------ boucles
    def _tick_positions(self):
        try:
            pos = self.bus.read_positions()
        except LeaderBusError as e:
            self._fails += 1
            if self._fails == FAILS_BEFORE_STALE:
                self._stale = True
                self.get_logger().error(
                    "Bus du leader perdu apres %d lectures en echec (%s). "
                    "Publication SUSPENDUE : mieux vaut pas de donnee qu'une donnee "
                    "perimee." % (self._fails, e)
                )
            return
        if self._stale:
            self.get_logger().info("Bus du leader retrouve, publication reprise.")
        self._fails, self._stale = 0, False
        self._dernier_pos = pos

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.name_of[sid] for sid in self.ids]
        msg.position = [steps_to_rad(pos[sid]) for sid in self.ids]
        self.pub_js.publish(msg)

    def _tick_telemetry(self):
        if self._stale:
            return
        try:
            tel = self.bus.read_telemetry()
        except LeaderBusError as e:
            self.get_logger().warn("Telemetrie illisible : %s" % e)
            return
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        for sid in self.ids:
            t = tel[sid]
            warns = telemetry_warnings(sid, t)
            for w in warns:
                self.get_logger().warn(w)
            st = DiagnosticStatus()
            st.name = "leader/%s" % self.name_of[sid]
            st.hardware_id = "sts3215_id%d" % sid
            st.level = DiagnosticStatus.WARN if warns else DiagnosticStatus.OK
            st.message = " ; ".join(warns) if warns else "OK"
            st.values = [
                KeyValue(key="tension_V",
                         value="%.1f" % (t["volt_dV"] / 10.0) if t["volt_dV"] is not None else "?"),
                KeyValue(key="temperature_C", value=str(t["temp_C"])),
                KeyValue(key="charge", value=str(t["load"])),
                KeyValue(key="couple", value="ON" if t["torque_on"] else "OFF"),
            ]
            arr.status.append(st)
        self.pub_tel.publish(arr)

    # ------------------------------------------------------------------ services
    def _sur_bus(self, service):
        """Un service qui touche le bus ne doit jamais tuer le noeud.

        Une lecture groupee ratee (un servo muet un instant) levait LeaderBusError hors
        de tout try : rclpy la relance, le noeud s'arretait, /leader/joint_states se
        taisait -- et /leader/recentrage restait a True. On coupe le couple (etat sur),
        on remet les modes a zero et on repond « echec » (revue du 2026-09-13)."""
        nom = getattr(service, "__name__", "service")

        @functools.wraps(service)
        def enveloppe(req, resp):
            try:
                return service(req, resp)
            except LeaderBusError as e:
                self.get_logger().error(
                    "bus en echec pendant %s : %s -> couple COUPE par securite" % (nom, e))
                self.maintien = False
                self._maintien_tient = False
                self.joy_actif = False
                for action in (lambda: self.bus.set_torque(False),
                               lambda: self.bus.set_torque_limit(TORQUE_LIMIT_MAX)):
                    try:
                        action()
                    except LeaderBusError:
                        pass
                self.pub_recentrage.publish(Bool(data=False))
                resp.success = False
                resp.message = "bus en echec : %s ; couple coupe par securite" % e
                return resp
        return enveloppe

    def _srv_set_torque_all(self, req, resp):
        echecs = self.bus.set_torque(req.data)
        resp.success = not echecs
        etat = "ACTIVE" if req.data else "coupe"
        resp.message = ("couple %s sur les %d servos" % (etat, len(self.ids))
                        if not echecs else "echec sur les servos %s" % echecs)
        if req.data:
            self.get_logger().warn("/leader/set_torque -> %s" % resp.message)
        else:
            self.get_logger().info("/leader/set_torque -> %s" % resp.message)
        return resp

    def _srv_set_torque_one(self, sid, req, resp):
        echecs = self.bus.set_torque(req.data, ids=[sid])
        resp.success = not echecs
        resp.message = ("couple %s sur %s (ID %d)"
                        % ("ACTIVE" if req.data else "coupe", self.name_of[sid], sid)
                        if not echecs else "echec sur l'ID %d" % sid)
        self.get_logger().info("/leader/%s/set_torque -> %s"
                               % (self.name_of[sid], resp.message))
        return resp

    def _apply_torque_limit(self, value, initial=False):
        echecs = self.bus.set_torque_limit(value)
        if echecs:
            self.get_logger().error("Limite de couple NON appliquee sur %s" % echecs)
            return False
        self.get_logger().info(
            "Limite de couple %s a %d/%d (%.0f %%)"
            % ("initialisee" if initial else "reglee", value, TORQUE_LIMIT_MAX,
               100.0 * value / TORQUE_LIMIT_MAX)
        )
        return True

    def _regler_couples(self, pcts):
        """Applique une liste de pourcentages (1 valeur = la meme partout)."""
        pcts = list(pcts)
        if len(pcts) == 1:
            pcts = pcts * len(self.ids)
        if len(pcts) != len(self.ids):
            return False, "%d valeurs pour %d axes" % (len(pcts), len(self.ids))
        if any(not 0.0 <= float(p) <= 100.0 for p in pcts):
            return False, "pourcentages hors de 0..100"
        self.joy_couple_max = {sid: int(TORQUE_LIMIT_MAX * float(p) / 100.0)
                               for sid, p in zip(self.ids, pcts)}
        self._joy_dernier_couple = {}   # force la reecriture au prochain cycle
        return True, ", ".join("%s %.0f%%" % (self.name_of[s], p)
                               for s, p in zip(self.ids, pcts))

    def _on_param(self, params):
        """La limite de couple se regle par parametre (donc par le service standard
        set_parameters) : c'est un reglage, pas une action. Utilise par US-022."""
        for p in params:
            if p.name == "joystick_pas_max":
                v = int(p.value)
                if not 10 <= v <= 2048:
                    return SetParametersResult(successful=False,
                                               reason="joystick_pas_max hors de 10..2048")
                self.joy_pas_max = v
                self.get_logger().warn("ecart de rappel max -> %d pas (%.0f deg)"
                                       % (v, v * 360.0 / 4096))
                continue
            if p.name == "joystick_torque_min_pct":
                vals = list(p.value)
                if len(vals) == 1:
                    vals = vals * len(self.ids)
                if len(vals) != len(self.ids) or any(not 0.0 <= float(v) <= 100.0 for v in vals):
                    return SetParametersResult(
                        successful=False, reason="joystick_torque_min_pct invalide")
                self.joy_couple_min = {sid: int(TORQUE_LIMIT_MAX * float(v) / 100.0)
                                       for sid, v in zip(self.ids, vals)}
                self._joy_dernier_couple = {}
                self.get_logger().warn("couple MIN joystick -> [%s]"
                                       % ", ".join("%s %.0f%%" % (self.name_of[s2], v)
                                                   for s2, v in zip(self.ids, vals)))
                continue
            if p.name == "joystick_deadzone_deg":
                # Reglable a CHAUD, comme le couple : on elargit la zone morte en
                # observant la derive a l'arret, sans relancer le noeud.
                try:
                    self.joy_deadzone = self._deadzones(p.value)
                except ValueError as e:
                    return SetParametersResult(successful=False, reason=str(e))
                self.get_logger().warn(
                    "zone morte joystick -> [%s]"
                    % ", ".join("%s %.1f deg" % (self.name_of[sid], math.degrees(z))
                                for sid, z in self.joy_deadzone.items()))
                continue
            if p.name == "joystick_torque_max_pct":
                # Reglable a CHAUD : on monte le couple par paliers en observant le bras,
                # sans relancer le noeud ni repasser par la sequence d'activation.
                ok, detail = self._regler_couples(p.value)
                if not ok:
                    return SetParametersResult(successful=False, reason=detail)
                self.get_logger().warn("couple joystick -> [%s]" % detail)
                continue
            if p.name != "torque_limit":
                continue
            if not 0 <= p.value <= TORQUE_LIMIT_MAX:
                return SetParametersResult(
                    successful=False,
                    reason="torque_limit hors plage 0..%d" % TORQUE_LIMIT_MAX,
                )
            if not self._apply_torque_limit(int(p.value)):
                return SetParametersResult(successful=False,
                                           reason="ecriture refusee par un servo")
        return SetParametersResult(successful=True)

    def _deadzones(self, valeurs):
        """-> {sid: zone morte en radians}. Une valeur = tous les axes."""
        v = [float(x) for x in list(valeurs)]
        if len(v) == 1:
            v = v * len(self.ids)
        if len(v) != len(self.ids):
            raise ValueError("joystick_deadzone_deg : %d valeurs pour %d axes"
                             % (len(v), len(self.ids)))
        if any(x < 0 or x > 90 for x in v):
            raise ValueError("joystick_deadzone_deg : valeur hors de 0..90 deg")
        return {sid: math.radians(x) for sid, x in zip(self.ids, v)}

    # -------------------------------------------------------------- calibration
    def _fichier_calib(self):
        if self._calib_chemin:
            return os.path.expanduser(self._calib_chemin)
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("roby_control"),
                            "config", "leader_calibration.yaml")

    def _mtime_calib(self):
        try:
            return os.path.getmtime(self._fichier_calib())
        except OSError:
            return None

    def _recharger_si_change(self):
        """Relit la calibration a chaud, comme les noeuds de teleoperation.

        Sans cela, le RECENTRAGE visait les zeros charges au demarrage du noeud : on
        deplacait la pose de repos du guide, on relancait le recentrage, et il revenait
        a l'ANCIENNE pose sans qu'aucun message ne le signale. Vecu le 2026-09-10.
        Le mode joystick est concerne de la meme facon, son rappel visant ces zeros.
        """
        m = self._mtime_calib()
        if m is None or m == self._mtime_calib_vu:
            return
        try:
            cal = charger_calib(self._calib_chemin)
        except Exception as e:
            self.get_logger().warn("calibration illisible, on garde l'ancienne : %s" % e)
            return
        self.cal, self._mtime_calib_vu = cal, m
        self.get_logger().warn(
            "calibration RECHARGEE : %s"
            % ", ".join("%s zero %.1f deg" % (j.nom_leader, math.degrees(j.zero))
                        for j in self.cal.joints))

    # ---------------------------------------------------------------- maintien
    def _srv_maintien(self, req, resp):
        if req.data and self.joy_actif:
            # Les deux ecrivent la meme cible avec des intentions opposees : l'un tire
            # vers le zero, l'autre veut rester sur place. On coupe le joystick.
            self._joystick_off()
            self.get_logger().warn("mode joystick coupe : incompatible avec le maintien")
        self.maintien = bool(req.data)
        # Lever le verrou : a partir d'ici le guide redevient manipulable, il tient
        # seulement quand on ne le touche pas.
        self.verrou = False
        self._maintien_cible = {}
        self._maintien_bouge = {}
        self._maintien_prec = None
        self._maintien_calme = 0
        self._maintien_tient = False
        self._maintien_libre = 0
        if self.maintien:
            pct = float(self.get_parameter("maintien_couple_pct").value)
            self.bus.set_torque_limit(int(TORQUE_LIMIT_MAX * pct / 100.0))
            self.bus.set_torque(True)
            # L'etat doit refleter la REALITE du couple : on vient de l'allumer, donc
            # on TIENT. Le laisser a False empechait la branche qui coupe le couple au
            # premier mouvement de se declencher -- le bras restait rigide et
            # impossible a manipuler apres l'embrayage.
            self._maintien_tient = True
            resp.message = ("maintien ACTIF a %.0f %% : le guide reste ou on le laisse, "
                            "et se deplace toujours a la main" % pct)
        else:
            self.bus.set_torque(False)
            self.bus.set_torque_limit(TORQUE_LIMIT_MAX)
            resp.message = "maintien coupe, bras libre (il s'affaissera)"
        resp.success = True
        self.get_logger().warn("/leader/maintien -> %s" % resp.message)
        return resp

    def _tick_maintien(self):
        # VERROU : entre le recentrage et l'embrayage, le guide ne doit pas bouger du
        # tout. On ne relache donc pas sur mouvement -- la pose recentree est un point
        # de depart, et un point de depart qui derive ne sert a rien. C'est l'embrayage
        # qui leve le verrou et rend le bras manipulable.
        if self.verrou:
            return
        """Assistance gravite : LIBRE quand on le deplace, TENU des qu'on le lache.

        Le premier essai baissait simplement la limite de couple. Insuffisant : tant
        que le servo est sous tension il POUSSE contre la main jusqu'a ce que sa cible
        rattrape, et l'operateur sent cette resistance meme a 5 % de couple. Le reglage
        du couple ne fait que doser la gene, il ne la supprime pas.

        On alterne donc franchement entre deux etats :
          - mouvement detecte  -> couple COUPE, le bras est totalement libre ;
          - immobile un instant -> cible = position courante, couple retabli, il tient.

        Le seul defaut est un leger affaissement a l'instant du lacher, le temps que
        l'immobilite soit constatee. C'est le prix de la compliance totale pendant le
        geste, et il est bien plus faible que la gene permanente d'un servo qui resiste.
        """
        if not self.maintien or self.joy_actif:
            return
        try:
            pos = self.bus.read_positions()
        except LeaderBusError:
            return
        prec = self._maintien_prec
        self._maintien_prec = dict(pos)
        if prec is None:
            return
        seuil = int(math.radians(
            float(self.get_parameter("maintien_relache_deg").value))
            * RESOLUTION / (2 * math.pi))
        bouge = any(abs(pas_court(prec.get(sid, p), p)) > max(1, seuil)
                    for sid, p in pos.items())
        hz = float(self.get_parameter("maintien_rate_hz").value)
        if bouge:
            self._maintien_calme = 0
            if self._maintien_tient:
                self.bus.set_torque(False)
                self._maintien_tient = False
                self._maintien_libre = 0
            self._maintien_libre += 1
            if (self.get_parameter("maintien_reprise_auto").value
                    and self._maintien_libre > max(1, int(
                        float(self.get_parameter("maintien_libre_max_s").value) * hz))):
                # Trop longtemps libre : on reprend la main plutot que de laisser une
                # eventuelle chute s'entretenir toute seule.
                self._reprendre(pos)
            return
        self._maintien_calme += 1
        if not self.get_parameter("maintien_reprise_auto").value:
            return          # une fois relache, il reste libre jusqu'au rearmement
        cycles = max(1, int(float(self.get_parameter("maintien_delai_s").value) * hz))
        if not self._maintien_tient and self._maintien_calme >= cycles:
            self._reprendre(pos)

    def _reprendre(self, pos):
        """Pose la cible sur la position courante et retablit le couple de tenue."""
        pct = float(self.get_parameter("maintien_couple_pct").value)
        self.bus.set_goal_position(pos)
        self.bus.set_torque_limit(int(TORQUE_LIMIT_MAX * pct / 100.0))
        self.bus.set_torque(True)
        self._maintien_tient = True
        self._maintien_libre = 0

    # --------------------------------------------------------------- recentrage
    def _srv_recentrer(self, req, resp):
        """Ramene le guide a sa pose de reference (les `zero` de la calibration).

        Pourquoi motorise plutot qu'a la main : replacer six axes au milieu de leur
        course a l'oeil est long et imprecis, et en teleoperation en POSITION une pose
        de depart fausse decale tout le suivi. Le bouton rend le point de depart
        reproductible.

        Deroulement : couple bas, on avance par petits pas vers le zero PAR LE PLUS
        COURT CHEMIN (le servo interpole en numero de pas, il ferait sinon presque un
        tour complet), puis on RECOUPE le couple -- l'etat de repos du guide reste
        "libre a la main", jamais "tenu par les moteurs".
        """
        if self.joy_actif:
            # On coupe le joystick NOUS-MEMES plutot que de refuser : les deux se
            # battraient pour la meme cible (l'un tire vers le zero, l'autre pilote la
            # position), et exiger de l'operateur qu'il pense a le couper d'abord est
            # une facon deguisee de lui faire porter un detail d'implementation.
            self._joystick_off()
            self.get_logger().warn("mode joystick coupe automatiquement pour le "
                                   "recentrage")
        g = self.get_parameter
        couple = int(TORQUE_LIMIT_MAX * float(g("recentrage_couple_pct").value) / 100.0)
        pas_cycle = int(g("recentrage_pas_par_cycle").value)
        tol = int(math.radians(float(g("recentrage_tolerance_deg").value))
                  * RESOLUTION / (2 * math.pi))
        timeout = float(g("recentrage_timeout_s").value)
        zeros = {}
        for j in self.cal.joints:
            sid = self._sid_de(j.nom_leader)
            if sid is not None and sid in self.ids:
                zeros[sid] = rad_to_steps(j.zero)
        if not zeros:
            resp.success = False
            resp.message = "aucun axe calibre a recentrer"
            return resp

        self.pub_recentrage.publish(Bool(data=True))
        self.bus.set_torque_limit(couple)
        self.bus.set_torque(True)
        t0 = time.monotonic()
        restants = dict(zeros)
        try:
            while time.monotonic() - t0 < timeout:
                pos = self.bus.read_positions()
                restants = {sid: z for sid, z in zeros.items()
                            if abs(pas_court(pos[sid], z)) > tol}
                if not restants:
                    break
                self.bus.set_goal_position({
                    sid: cible_de_rappel(pos[sid], z, pas_cycle, 0)
                    for sid, z in restants.items()})
                time.sleep(0.05)
        except LeaderBusError as e:
            self.bus.set_torque(False)
            self.bus.set_torque_limit(TORQUE_LIMIT_MAX)
            self.pub_recentrage.publish(Bool(data=False))
            resp.success = False
            resp.message = "bus en echec pendant le recentrage : %s" % e
            return resp
        # Etat de repos. Deux cas, selon `recentrage_maintien_pct` :
        #   0   -> couple coupe, bras libre a la main (defaut historique)
        #   > 0 -> le guide TIENT sa pose ; on ecrit la cible avant de baisser le couple,
        #          sinon le servo garde l'ancienne consigne et repart aussitot.
        # Le recentrage ACTIVE le maintien : entre le moment ou le guide rejoint sa
        # pose de reference et celui ou l'operateur embraye, il doit tenir cette pose.
        # Sans cela il s'affaisse pendant l'intervalle, et la pose soigneusement
        # recentree est perdue avant meme d'avoir servi.
        self.maintien = True
        self.verrou = True
        # La machine a etats du maintien tourne en parallele : sans cette remise a
        # zero, elle voit le bras bouger pendant le recentrage, en conclut que
        # l'operateur le deplace, et coupe le couple juste apres.
        self._maintien_prec = None
        self._maintien_calme = 10 ** 6
        self._maintien_tient = True
        self._maintien_libre = 0
        maintien = float(self.get_parameter("recentrage_maintien_pct").value)
        if maintien > 0:
            try:
                pos = self.bus.read_positions()
                self.bus.set_goal_position({sid: pos[sid] for sid in zeros
                                            if sid in pos})
            except LeaderBusError:
                pass
            self.bus.set_torque_limit(
                int(TORQUE_LIMIT_MAX * maintien / 100.0))
        else:
            self.bus.set_torque(False)
            self.bus.set_torque_limit(TORQUE_LIMIT_MAX)
        if restants:
            noms = ", ".join(self.name_of[sid] for sid in restants)
            resp.success = False
            resp.message = ("recentrage INCOMPLET apres %.0f s : %s n'ont pas atteint "
                            "leur zero (butee, frottement, ou couple trop bas)"
                            % (timeout, noms))
        else:
            resp.success = True
            resp.message = ("guide recentre en %.1f s (%d axes), VERROUILLE "
                            "(l'embrayage le liberera), %s"
                            % (time.monotonic() - t0, len(zeros),
                               ("MAINTENU a %.0f %% de couple" % maintien) if maintien > 0
                               else "couple recoupe : bras libre"))
        self.pub_recentrage.publish(Bool(data=False))
        self.get_logger().warn("/leader/recentrer -> %s" % resp.message)
        return resp

    # ------------------------------------------------------------------ joystick
    def _srv_joystick(self, req, resp):
        if req.data:
            ok, msg = self._joystick_on()
        else:
            ok, msg = self._joystick_off()
        resp.success, resp.message = ok, msg
        # Deux appels DISTINCTS : rclpy indexe ses journaux par site d'appel et leve
        # "Logger severity cannot be changed between calls" si un meme site alterne
        # entre warn et info. Vecu le 2026-09-05 -> le noeud plantait a la COUPURE du
        # mode, c'est-a-dire au pire moment : un crash laisse le couple actif.
        if req.data:
            self.get_logger().warn("/leader/joystick -> %s" % msg)
        else:
            self.get_logger().info("/leader/joystick -> %s" % msg)
        return resp

    def _joystick_on(self):
        if self.cal is None or self.cal.non_calibres():
            return False, "calibration absente ou incomplete : mode refuse"
        if self._stale or self._dernier_pos is None:
            return False, "positions indisponibles : mode refuse"
        # Sequence en 3 temps, pour qu'aucun a-coup ne soit possible :
        #   1. couple ON en figeant la cible sur la position ACTUELLE (aucun mouvement)
        #   2. couple plafonne au minimum
        #   3. cible = zero -> le bras part vers le neutre, doucement
        echecs = self.bus.set_torque(True)
        if echecs:
            return False, "couple non active sur %s" % echecs
        self.bus.set_torque_limit(0)
        # La cible est desormais calculee a chaque cycle (elle SUIT le bras) : on ne vise
        # plus jamais le zero absolu, qui faisait partir l'epaule et le poignet par le
        # chemin long, soit 71 % de tour a l'envers (2026-09-05).
        self._joy_dernier_couple = {}
        self._joy_prev = None
        self.joy_actif = True
        detail = ", ".join("%s %.0f%%" % (self.name_of[sid],
                                          100.0 * c / TORQUE_LIMIT_MAX)
                           for sid, c in self.joy_couple_max.items())
        return True, ("mode joystick ACTIF : rappel vers le zero par le plus court "
                      "chemin, zone morte par axe [%s], couple max par axe [%s]"
                      % (", ".join("%s %.1f deg" % (self.name_of[sid],
                                                    math.degrees(z))
                                   for sid, z in self.joy_deadzone.items()),
                         detail))

    def _joystick_off(self):
        self.joy_actif = False
        self.bus.set_torque_limit(TORQUE_LIMIT_MAX)
        echecs = self.bus.set_torque(False)
        return (not echecs), ("mode joystick coupe, bras libre"
                              if not echecs else "couple NON coupe sur %s" % echecs)

    def _sid_de(self, nom_leader):
        for sid, nom in self.name_of.items():
            if nom == nom_leader:
                return sid
        return None

    def _tick_joystick(self):
        if not self.joy_actif or self._dernier_pos is None:
            return
        pos, t = self._dernier_pos, self.get_clock().now().nanoseconds * 1e-9

        # --- garde anti-emballement ---
        if self._joy_prev is not None:
            dt = t - self._joy_prev[1]
            if dt > 1e-3:
                for sid, p in pos.items():
                    dp = abs((steps_to_rad(p) - steps_to_rad(self._joy_prev[0][sid])
                              + math.pi) % (2 * math.pi) - math.pi)
                    if dp / dt > self.joy_vmax:
                        self._joystick_off()
                        self.get_logger().error(
                            "EMBALLEMENT sur l'ID %d (%.0f deg/s > %.0f) : mode joystick "
                            "COUPE par securite." % (sid, math.degrees(dp / dt),
                                                     math.degrees(self.joy_vmax)))
                        return
        self._joy_prev = (dict(pos), t)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        cibles = {}
        for j in self.cal.joints:
            sid = self._sid_de(j.nom_leader)
            if sid is None or sid not in pos:
                continue
            v, couple = commande_axe(j, steps_to_rad(pos[sid]), self.joy_deadzone[sid],
                                     self.joy_couple_max[sid],
                                     self.joy_couple_min.get(sid, 0))
            msg.name.append(j.nom_urdf)
            msg.velocity.append(v)
            # Cible recalculee a chaque cycle : elle suit le bras, toujours dans la
            # direction du plus court chemin vers le zero, et jamais a plus de
            # `joystick_pas_max`.
            cible = cible_de_rappel(
                pos[sid], rad_to_steps(j.zero), self.joy_pas_max,
                int(self.joy_deadzone[sid] * 4096 / (2 * math.pi)))
            cibles[sid] = cible
            # On n'ecrit le couple que s'il change : inutile de saturer le bus.
            if self._joy_dernier_couple.get(sid) != couple:
                self.bus.set_torque_limit(couple, ids=[sid])
                self._joy_dernier_couple[sid] = couple
        if cibles:
            self.bus.set_goal_position(cibles)
        self.pub_joy.publish(msg)

    # ------------------------------------------------------------------ arret
    def shutdown(self):
        """Couple coupe AVANT fermeture du port — y compris sur Ctrl-C."""
        try:
            self.joy_actif = False
            self.bus.set_torque_limit(TORQUE_LIMIT_MAX)
            echecs = self.bus.set_torque(False)
            if echecs:
                self.get_logger().error(
                    "ATTENTION : couple NON coupe sur %s a l'arret !" % echecs
                )
            else:
                self.get_logger().info("Couple coupe, bras guide libre a la main.")
        except Exception as e:  # noqa: BLE001 - on ferme le port quoi qu'il arrive
            self.get_logger().error("Coupure du couple impossible a l'arret : %s" % e)
        finally:
            self.bus.close()


def main(args=None):
    iface = verifier_interface_dds()
    print("[leader_node] interface DDS validee : %s (lien cable du projet)" % iface,
          flush=True)
    rclpy.init(args=args)
    node = None
    try:
        node = LeaderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
