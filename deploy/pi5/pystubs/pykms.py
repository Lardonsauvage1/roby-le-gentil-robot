# stub : picamera2 importe pykms (preview DRM) qu on n utilise pas. __getattr__
# rend un Mock pour toute reference, l import passe, jamais appele reellement.
from unittest.mock import MagicMock
def __getattr__(name): return MagicMock()
