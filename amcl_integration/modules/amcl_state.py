"""
modules/amcl_state.py
─────────────────────────────────────────────────────
Shared mutable state specific to AMCL localization.
Import as: import modules.amcl_state as amcl_state
"""

import threading

# Lock for AMCL specific state
amcl_lock = threading.Lock()

AMCL_ACTIVE: bool = False
AMCL_CONVERGED: bool = False
# 6x6 covariance matrix diagonal from /amcl_pose. Used to determine convergence.
AMCL_COVARIANCE: list | None = None
AMCL_INITIAL_POSE_SET: bool = False
LOCALIZATION_MODE: str = "slam"  # "slam" | "amcl"
