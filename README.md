# AMCL Integration for RoboPi

This folder contains the standalone files required to integrate Adaptive Monte Carlo Localization (AMCL) into the RoboPi project at runtime. Currently, the RoboPi project supports AMCL only at launch time via `use_slam:=false`. These scripts allow dynamically switching from SLAM to AMCL from the web dashboard.

## Overview of Files

- **`amcl_launch.py`**: A dedicated ROS 2 launch file that starts ONLY the AMCL node and Map Server. It uses `nav2_lifecycle_manager` to handle their lifecycles independently of the main navigation stack.
- **`amcl_params.yaml`**: Dedicated parameter tuning for AMCL, extracted from the main nav params, with higher particle counts for dead-reckoning environments.
- **`amcl_manager.py`**: A Python module (to be placed in `modules/`) that acts as a bridge. It spawns `amcl_launch.py` as a subprocess when AMCL mode is requested and kills it when returning to SLAM.
- **`amcl_state.py`**: A state holder (to be placed in `modules/`) for AMCL-specific variables (active status, covariance, etc.) without cluttering the main `state.py`.
- **`amcl_routes.py`**: A FastAPI router exposing REST endpoints (`/api/amcl/start`, `/api/amcl/initial_pose`, etc.) to control AMCL from the web dashboard.
- **`dashboard_amcl_patch.html`**: HTML and JavaScript snippets required to add AMCL start/stop buttons and map-click "Initial Pose" functionality to your `templates/index.html` dashboard.

## Integration Instructions

1. Copy `amcl_launch.py` and `amcl_params.yaml` to the root of your RoboPi control directory.
2. Copy `amcl_manager.py` and `amcl_state.py` into your `modules/` directory.
3. Copy `amcl_routes.py` to the root of your RoboPi control directory.
4. Open `dashboard_amcl_patch.html` and follow the instructions within to insert the UI controls and JavaScript into your `templates/index.html`.
5. Modify `pythonserver.py` to include the AMCL routes and handle shutdown:

```python
# Add to imports
from amcl_routes import amcl_router
import modules.amcl_state as amcl_state
from modules.amcl_manager import stop_amcl

# Add to FastAPI app setup
app.include_router(amcl_router)

# Add to lifespan shutdown section
stop_amcl()

# Add to telemetry broadcast in _telemetry_loop
await manager.broadcast({
    "event": "telemetry",
    "data": {
        **chassis.get_telemetry(),
        "nav_mode": state.NAV_MODE,
        "amcl_active": amcl_state.AMCL_ACTIVE,
        "amcl_converged": amcl_state.AMCL_CONVERGED,
        # ... other existing telemetry fields ...
    },
})
```
