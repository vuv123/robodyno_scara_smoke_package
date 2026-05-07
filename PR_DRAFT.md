# Add smoke coverage for installed Robodyno model library

## Summary

- Add formal smoke tests for the installed Robodyno model classes.
- Cover `ThreeDoFCartesian`, `ThreeDoFPallet`, `FourDoFPallet`, `ThreeDoFDelta`, and `SixDoFCollabRobot`.
- Validate FK/IK behavior, finite numerical outputs, and basic joint lifecycle methods.
- Preserve the existing Webots smoke fix and full PROTO loading coverage.

## Validation

- `python -m unittest tests.test_installed_robodyno_models -v`
  - Result: `6/6 OK`
- `python -B -m robodyno_scara_smoke.run_all_smoke_tests`
  - Result: `44/44 OK`
- Webots full PROTO loading smoke passed.
  - Log: `all_protos_smoke_output.txt`
- SCARA Webots motion smoke passed.
  - Log: `scara_smoke_result.txt`

## Notes

- Webots emits non-fatal warnings about large meshes, default controller directories, and OpenGL fallback behavior.
- No `ERROR` entries were observed in the final smoke logs.
