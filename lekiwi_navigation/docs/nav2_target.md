# Nav2 target tracker: validation and tests

## Validation on a ROS 2 device

After building and sourcing the package on the robot:

1. Start normal robot bringup and confirm the sphere and axes coincide with the base in
   teleop, including while driving. There must be no map-TF requirement in teleop.
2. Switch to Nav2 with the existing mode button. Confirm the target preserves its
   position, then stays map-fixed without input.
3. Move and rotate it with the existing joystick/Foxglove controls. Rotate 90
   degrees and verify forward translation follows the target's X axis.
4. Release the controls and confirm the target stops; switch modes with a held
   command and check that no old command produces a jump.
5. Return to teleop and verify reattachment. Inspect TF around both parent changes
   for lookup errors or transient display jumps.
6. Restart the tracker after selecting Nav2, restart the twist switch, and test a
   transition with map TF unavailable. Check the reported state and recovery after
   a new successful mode call / restored TF.
7. Confirm editing alone never submits a goal. Call `/nav2_send_goal` with false
   (no-op while idle), then true in teleop (rejected), then true in Nav2 mode (submitted).
8. Compare `nav2_target_submitted_pose` against the draft at submission, including
   heading. Edit the draft during navigation and verify the submitted goal stays
   unchanged. A second true call while busy must be rejected.
9. Check success, rejection, failure, external cancellation, and subsequent
   submissions. Switch to teleop during execution and confirm the action is not
   canceled by this node. Check the existing patrol detour/resume behavior.

## Automated target tests

After building and sourcing on a ROS 2 device, run from the repository root:

```bash
pytest lekiwi_navigation/test/test_nav2_target*.py lekiwi_audio/test/test_indicator_node.py -q
```

A fake Nav2 action server exercises the service and action round trip, so no robot is needed; run it on an isolated ROS domain without a running navigation stack. The source asset checks (button binding, launch wiring, defaults, phrases) also run without ROS, with PyYAML installed:

```bash
python lekiwi_navigation/test/test_nav2_target_assets.py
```

TF display across parent changes, real patrol interaction, controller input and audio playback need the robot validation above.
