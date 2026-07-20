#!/usr/bin/env python3
"""Serveur UDP d'echo de temps (sur le Pi5) : renvoie time.time() du Pi5. Sert a mesurer
le decalage d'horloge PC<->Pi5 (type NTP) sans rien installer. Port 9999."""
import socket, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("0.0.0.0", 9999))
while True:
    data, addr = s.recvfrom(64)
    s.sendto(repr(time.time()).encode(), addr)
