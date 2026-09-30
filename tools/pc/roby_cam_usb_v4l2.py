#!/usr/bin/env python3
"""roby_cam_usb_v4l2.py — lecture / écriture des contrôles V4L2 d'une caméra USB (UVC).

Parle directement au noyau (ioctl), donc AUCUNE dépendance à `v4l2-ctl` (pas installé
sur le PC, et son installation demande sudo). Utilisé par `roby_cam_usb.py`.

Usage autonome :
  python3 roby_cam_usb_v4l2.py                  # liste les contrôles de la caméra USB
  python3 roby_cam_usb_v4l2.py --apply f.json   # applique un préréglage enregistré
"""
import fcntl
import json
import os
import struct
import sys

# Codes ioctl (linux/videodev2.h)
VIDIOC_QUERYCTRL = 0xC0445624
VIDIOC_G_CTRL = 0xC008561B
VIDIOC_S_CTRL = 0xC008561C
VIDIOC_QUERYMENU = 0xC02C5625
FLAG_NEXT_CTRL = 0x80000000
FLAG_INACTIVE = 0x10
TYPES = {1: "int", 2: "bool", 3: "menu", 4: "button", 6: "class", 9: "intmenu"}

# Nom sous lequel la caméra OV4689 4MP (SKU 34767) s'annonce.
DEFAULT_NAME = "AK-Camera"

# Ordre d'application d'un préréglage : les interrupteurs « auto » AVANT les valeurs
# manuelles, sinon le noyau refuse la valeur (contrôle inactif tant que l'auto est ON).
AUTO_FIRST = ("auto_exposure", "exposure_dynamic_framerate",
              "white_balance_automatic", "focus_automatic_continuous")


def find_device(name=DEFAULT_NAME):
    """Retourne /dev/videoN de la caméra dont le nom contient `name` (flux image, pas métadonnées)."""
    base = "/sys/class/video4linux"
    for d in sorted(os.listdir(base), key=lambda s: int(s[5:])):
        try:
            with open(f"{base}/{d}/name") as f:
                nm = f.read().strip()
            with open(f"{base}/{d}/index") as f:
                idx = int(f.read())
        except OSError:
            continue
        if name in nm and idx == 0:
            return f"/dev/{d}"
    return None


def slug(name):
    """'Exposure Time, Absolute' -> 'exposure_time_absolute' (même convention que v4l2-ctl)."""
    out = "".join(c.lower() if c.isalnum() else "_" for c in name)
    while "__" in out:
        out = out.replace("__", "_")
    return out.strip("_")


class Controls:
    def __init__(self, dev):
        self.dev = dev
        self.fd = os.open(dev, os.O_RDWR)

    def close(self):
        os.close(self.fd)

    def reopen(self, dev):
        """Après une reconnexion USB (la caméra peut changer de /dev/videoN)."""
        old, self.dev, self.fd = self.fd, dev, os.open(dev, os.O_RDWR)
        try:
            os.close(old)
        except OSError:
            pass

    def _get(self, cid):
        buf = bytearray(struct.pack("Ii", cid, 0))
        fcntl.ioctl(self.fd, VIDIOC_G_CTRL, buf)
        return struct.unpack("Ii", buf)[1]

    def _menu(self, cid, mn, mx):
        items = {}
        for k in range(mn, mx + 1):
            q = bytearray(struct.pack("II32sI", cid, k, b"", 0))
            try:
                fcntl.ioctl(self.fd, VIDIOC_QUERYMENU, q)
            except OSError:
                continue
            items[k] = struct.unpack("II32sI", q)[2].split(b"\0")[0].decode()
        return items

    def list(self):
        """Liste des contrôles : dicts {id, key, name, type, min, max, step, default, value, inactive, menu}."""
        out, cid = [], FLAG_NEXT_CTRL
        while True:
            buf = bytearray(struct.pack("II32siiiiI8x", cid, 0, b"", 0, 0, 0, 0, 0))
            try:
                fcntl.ioctl(self.fd, VIDIOC_QUERYCTRL, buf)
            except OSError:
                break
            i, t, name, mn, mx, st, df, fl = struct.unpack("II32siiiiI8x", buf)
            cid = i | FLAG_NEXT_CTRL
            typ = TYPES.get(t, str(t))
            if typ not in ("int", "bool", "menu"):
                continue
            name = name.split(b"\0")[0].decode()
            try:
                val = self._get(i)
            except OSError:
                val = None
            out.append({"id": i, "key": slug(name), "name": name, "type": typ,
                        "min": mn, "max": mx, "step": st, "default": df, "value": val,
                        "inactive": bool(fl & FLAG_INACTIVE),
                        "menu": self._menu(i, mn, mx) if typ == "menu" else None})
        return out

    def by_key(self):
        return {c["key"]: c for c in self.list()}

    def set(self, key, value):
        """Écrit un contrôle, puis RELIT la valeur réellement retenue par la caméra."""
        c = self.by_key()[key]
        buf = bytearray(struct.pack("Ii", c["id"], int(value)))
        fcntl.ioctl(self.fd, VIDIOC_S_CTRL, buf)
        return self._get(c["id"])

    def snapshot(self):
        return {c["key"]: c["value"] for c in self.list()}

    def apply(self, preset):
        """Applique un préréglage {key: value} (autos d'abord). Retourne les écarts constatés."""
        ecarts = {}
        for k in AUTO_FIRST:
            if k in preset:
                got = self.set(k, preset[k])
                if got != preset[k]:
                    ecarts[k] = (preset[k], got)
        # Relire APRÈS les autos : un contrôle encore inactif (ex. température de blanc
        # alors que la balance auto est restée ON) serait refusé par le noyau.
        ctrls = self.by_key()
        for k, v in preset.items():
            if k in AUTO_FIRST or k not in ctrls or ctrls[k]["inactive"]:
                continue
            try:
                got = self.set(k, v)
            except OSError as e:
                got = f"refusé ({e.strerror})"
            if got != v:
                ecarts[k] = (v, got)
        return ecarts


def main():
    dev = find_device()
    if dev is None:
        sys.exit(f"Caméra « {DEFAULT_NAME} » introuvable (branchée ? `lsusb`)")
    ctl = Controls(dev)
    if len(sys.argv) == 3 and sys.argv[1] == "--apply":
        with open(sys.argv[2]) as f:
            preset = json.load(f)["controls"]
        ecarts = ctl.apply(preset)
        print(f"{dev} : préréglage appliqué" + (f", ÉCARTS {ecarts}" if ecarts else ", relu identique"))
        sys.exit(1 if ecarts else 0)
    print(dev)
    for c in ctl.list():
        extra = f"  {c['menu']}" if c["menu"] else ""
        ina = "  [inactif]" if c["inactive"] else ""
        print(f"  {c['key']:34s} {c['type']:5s} {c['min']}..{c['max']} défaut={c['default']} "
              f"actuel={c['value']}{ina}{extra}")


if __name__ == "__main__":
    main()
