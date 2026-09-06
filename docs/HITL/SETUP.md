# HITL Setup

1. Connect the batteries on the drone to power the Herelink.
2. Turn on the Herelink ground station controller and enable its hotspot.
3. Connect the laptop to the Herelink Wi-Fi.
4. Open QGroundControl. Go to **Configure > Sensors** and complete each calibration.
5. Open **Fly** and set the gimbal pitch to 90 degrees. Alternatively, open the console and run:

   ```text
   gimbal test pitch -90
   ```

6. Press the camera power button, then unplug and reconnect the Elgato.
7. Connect to the Jetson:

   ```bash
   ssh bvorinnano@192.168.144.2
   ```

8. Follow the camera and PX4 checks in [run_mission.md](run_mission.md), then stop unnecessary overhead:
9. Check the real_params.yaml coordinates to make sure they are correct. Use [Scan_Picker.py](../../scripts/Scan_Picker.py)