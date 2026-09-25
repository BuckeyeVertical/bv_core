# Testing in the simulator

Run ROS commands in the container:

```bash
docker start bv-mission
docker exec bv-mission bash -c "source /opt/bv/ros_setup.sh && cd /bv_ws && <command>"
```

- Build: `colcon build`
- Test: `colcon test && colcon test-result --verbose`

## Full mission in SITL

Start each as a background process, in order (from `~/Code/bv_bevy/docs/Run.md`):

1. PX4 + Gazebo: `cd ~/Code/bv_bevy && docker compose -f gazebo/compose.px4.yaml up`
2. Bevy: `cd ~/Code/bv_bevy && ./run_suas.sh`
3. MAVROS (container): `ros2 launch mavros px4.launch fcu_url:=udp://:14540@host.docker.internal:14580`
4. Mission (container): `BV_MISSION_CONFIG=sim_params.yaml ros2 launch bv_core mission.launch.py`

GCS: http://localhost:8765. Stop everything when done.
