Robotics to-do
#TimberX

New
Calibrate the webcam (FOCAL_PX)
1. Deploy the current code to the robot (Android Studio → Run 'TeamCode').
2. Connect to the robot's network (Control Hub Wi‑Fi or USB tether).
3. Open FTC Dashboard in a browser: usually http://192.168.43.1:8080/dash on the Hub's own Wi‑Fi AP, or http://192.168.42.1:8080/dash if tethered by USB.
4. On the Driver Station, select VisionCalibration → INIT. In the Dashboard's Configuration tab, find VisionCalibration and set VIEW = 0 (webcam). Press START.
5. Hold a yellow ball so its center is exactly 12″ from the webcam lens (measure lens → ball center, not from the bumper). Keep it well lit and near the middle of the camera view.
    - If the real distance isn't 12″, set VisionCalibration.CALIBRATE_DISTANCE_IN in the Dashboard to the actual distance first — suggestFocal uses that number.
6. Read the DS telemetry. It will show something like suggestFocal = 612. In the Dashboard Configuration, find PollenVision and set FOCAL_PX to that value, press Enter.
7. Verify: telemetry range should now read ≈12″. Repeat at 6″, 24″, 36″. If it's consistently off, measure the real ball diameter and set PollenVision.TARGET_DIAM_IN to it, then redo step 5.
8. If FPS is low or detection is noisy: set RESOLUTION_WIDTH = 320, RESOLUTION_HEIGHT = 240, and ENABLE_RED = false / ENABLE_BLUE = false. Then toggle VisionCalibration.REBUILD 0→1→0 to re-open the portal, and recalibrate FOCAL_PX (focal length scales with resolution).
9. Only then test the held‑A assist, with PollenIntakeAssist.STOP_RANGE_IN set to wherever the ball should stop relative to the intake. Have your finger ready to release A.

Load a BioBuzz AprilTag map into the Limelight
This is a Limelight web-UI task — Java cannot supply the tag map, and there's no BioBuzz map in the code:
1. Get the official field map: the BioBuzz AprilTag layout JSON (tag IDs + field coordinates) from FIRST's official game/field CAD resources. I don't have it and won't invent IDs — without it, real tag poses aren't possible, so relocalization stays deferred.
2. Reach the Limelight UI: power the robot, join the Limelight's own Wi‑Fi network (usually named limelight), and open http://limelight.local:5801 in a browser (some firmware uses http://172.29.12.1:5801).
3. In the UI, find the AprilTag / Field Map / Tag Map section (label varies by firmware), upload or paste the JSON, save, and reboot/reload the pipeline.
4. Make sure the AprilTag pipeline (index 1 — StudioAprilTag.APRILTAG_PIPELINE) is in fiducial/AprilTag detection mode.
5. Set the camera pose in that same UI: where the Limelight is physically mounted on the robot (x/y/z offsets from robot center, yaw/pitch). Without this, botpose is garbage even with a valid map.
6. Verify: run VisionCalibration with VIEW = 1. Point at a tag — you should see tag IDs and a plausible botpose. IDs but nonsense pose = bad map or camera pose; no IDs = wrong pipeline mode or lighting.

- Important: even after all that, the robot's pose estimate still comes from Pinpoint only. Loading the map just makes the observations meaningful — the setPose(...) relocalization we deferred is a separate step.
- If you can find the BioBuzz map JSON (or just the tag IDs and coordinates), paste it here and I can wire the coordinate conversion into the code whenever you're ready to turn relocalization on.

Specs
Field poses (Pedro coordinates — inches, degrees; format x, y, heading°):
* Start pose:
* Pickup pose(s) — where each object sits:
* Scoring pose:
* Park pose (if any):
* How many cycles?

Limelight setup:
* Is an AprilTag field map (.fmap) uploaded in the Limelight web UI? (required for botpose)
* Which pipeline is the AprilTag one (0–9)?
* Hardware map name — just limelight?

Scoring mechanism:
* There are leftIntake, rightIntake, intakeWheel, and rubber. Which of these should run during pickup vs. score, and at what powers?
