# loader for defaults or local

from .config_defaults import *

try:
    from .config_local import *
except ImportError:
    pass