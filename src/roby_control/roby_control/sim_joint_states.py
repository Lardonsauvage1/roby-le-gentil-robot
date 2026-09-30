"""Publication de /joint_states par un bras SIMULE, jamais a cote d'un vrai robot.

BUG-008 (2026-09-10) : un noeud de teleoperation simulee, reste vivant 15 h sur le
domaine 42, publiait la pose du bras simule sur /joint_states en meme temps que le
joint_state_broadcaster du Pi5. Le jog recopiait cette pose fausse : 38 deg de saut sur
l'axe 1 du vrai bras. Les correctifs d'alors etaient procéduraux (balayage au lancement,
GATE) ; celui-ci est dans le code.

Regle : un bras simule ne publie que s'il est SEUL a publier /joint_states. Un autre
publisher, c'est un robot (ou un second simulateur) : on se tait et on le dit. Pour
simuler a cote de la vraie stack, utiliser un autre domaine (ROS_DOMAIN_ID=43).
"""

from __future__ import annotations

import time

TOPIC = "/joint_states"


class GardeJointStates:
    """Filtre les publications /joint_states d'un noeud de SIMULATION."""

    def __init__(self, node, publisher, attente_s: float = 2.0, periode_s: float = 0.5):
        self.node = node
        self.pub = publisher
        # Silence au demarrage : la decouverte DDS du graphe prend du temps, et les
        # premiers messages partiraient avant qu'on voie le vrai robot.
        self.attente_s = attente_s
        self.periode_s = periode_s
        self._t0 = time.monotonic()
        self._t_verif = -1e9
        self._autres = 0
        self._signale = None

    def autres_publishers(self) -> int:
        return max(0, self.node.count_publishers(TOPIC) - 1)

    def publier(self, msg) -> bool:
        """Publie `msg` si le noeud est seul sur /joint_states. Renvoie True si publie."""
        maintenant = time.monotonic()
        if maintenant - self._t_verif >= self.periode_s:
            self._t_verif = maintenant
            self._autres = self.autres_publishers()
            if bool(self._autres) != self._signale:
                self._signale = bool(self._autres)
                if self._autres:
                    self.node.get_logger().error(
                        "%d autre(s) publisher(s) de %s (vraie stack ?) : ce bras SIMULE "
                        "se TAIT pour ne pas fausser l'etat du robot (BUG-008). Simuler "
                        "sur un autre domaine : ROS_DOMAIN_ID=43." % (self._autres, TOPIC))
                elif maintenant - self._t0 >= self.attente_s:
                    self.node.get_logger().info("seul publisher de %s : simulation publiee"
                                                % TOPIC)
        if self._autres or maintenant - self._t0 < self.attente_s:
            return False
        self.pub.publish(msg)
        return True
