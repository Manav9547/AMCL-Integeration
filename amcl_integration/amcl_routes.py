"""
amcl_routes.py
─────────────────────────────────────────────────────
FastAPI routes for controlling AMCL localization.
"""

from fastapi import APIRouter
from fastapi.responses import JSONResponse
from pydantic import BaseModel
import os

from modules import state
import modules.amcl_state as amcl_state
from modules import amcl_manager
from modules.ws_manager import ConnectionManager

amcl_router = APIRouter()

# Note: You might need to import your existing manager if you want to broadcast
# from modules.pythonserver import manager
# For this script, we'll assume manager is available or we return HTTP responses.

class AMCLStartRequest(BaseModel):
    map_name: str

@amcl_router.post("/api/amcl/start")
async def start_amcl_route(req: AMCLStartRequest):
    """Start AMCL with a saved map."""
    maps_dir = os.path.expanduser("~/robopi_maps")
    
    # Allow passing either 'my_map' or 'my_map.yaml'
    yaml_name = req.map_name if req.map_name.endswith(".yaml") else f"{req.map_name}.yaml"
    map_yaml_path = os.path.join(maps_dir, yaml_name)
    
    if not os.path.exists(map_yaml_path):
        return JSONResponse(
            {"status": "error", "message": f"Map not found: {yaml_name}"},
            status_code=404
        )
        
    success = amcl_manager.start_amcl(map_yaml_path)
    
    if success:
        # Also load the map visually for the dashboard
        from modules import ros_node
        ros_node.load_map_from_disk(map_yaml_path)
        
        return JSONResponse({
            "status": "started",
            "message": "AMCL launched successfully.",
            "map": yaml_name
        })
    else:
        return JSONResponse(
            {"status": "error", "message": "Failed to start AMCL."},
            status_code=500
        )

@amcl_router.post("/api/amcl/stop")
async def stop_amcl_route():
    """Stop AMCL and revert to SLAM mapping."""
    success = amcl_manager.stop_amcl()
    if success:
        return JSONResponse({"status": "stopped", "message": "AMCL stopped, SLAM mapping resumed."})
    else:
        return JSONResponse(
            {"status": "error", "message": "Failed to stop AMCL."},
            status_code=500
        )

@amcl_router.get("/api/amcl/status")
async def get_amcl_status():
    """Get the current status of AMCL."""
    with amcl_state.amcl_lock:
        return JSONResponse({
            "active": amcl_state.AMCL_ACTIVE,
            "mode": amcl_state.LOCALIZATION_MODE,
            "initial_pose_set": amcl_state.AMCL_INITIAL_POSE_SET,
            "converged": amcl_state.AMCL_CONVERGED
        })

class InitialPoseRequest(BaseModel):
    x: float
    y: float
    yaw: float

@amcl_router.post("/api/amcl/initial_pose")
async def set_initial_pose_route(req: InitialPoseRequest):
    """Set the initial pose estimate for AMCL."""
    success = amcl_manager.set_initial_pose(req.x, req.y, req.yaw)
    if success:
        return JSONResponse({"status": "ok", "message": "Initial pose sent."})
    else:
        return JSONResponse(
            {"status": "error", "message": "Failed to set initial pose. Is AMCL running?"},
            status_code=500
        )

@amcl_router.post("/api/amcl/reinitialize")
async def reinitialize_amcl_route():
    """Trigger global relocalization (scatter particles)."""
    success = amcl_manager.reinitialize_global_localization()
    if success:
        return JSONResponse({"status": "ok", "message": "Global localization triggered."})
    else:
        return JSONResponse(
            {"status": "error", "message": "Failed to trigger global localization."},
            status_code=500
        )
