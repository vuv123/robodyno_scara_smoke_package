# robodyno SCARA smoke package

This repository verifies the Robodyno SCARA/Webots smoke workflow against the installed `robodyno` package and local Webots assets.

## What this verifies

- `robodyno_scara_smoke.ScaraGeometry`
- `robodyno_scara_smoke.create_scara(joints, geometry=None)`
- `robodyno_scara_smoke.create_webots_scara(webots, geometry=None)`
- `python -m robodyno_scara_smoke.run_webots_smoke`
- `python -m robodyno_scara_smoke.run_webots_proto_smoke`
- `python -m robodyno_scara_smoke.run_all_smoke_tests`

## Run tests

```powershell
cd D:\smoke\robodyno_scara_smoke_package
python -m unittest discover -s tests
python -m robodyno_scara_smoke.run_webots_smoke
python -m robodyno_scara_smoke.run_webots_proto_smoke
python -m robodyno_scara_smoke.run_all_smoke_tests
python -m robodyno_scara_smoke.run_model_stress_tests --iterations 500
```

## Notes

- `all_protos_smoke_output.txt` stores the Webots PROTO smoke output.
- `scara_smoke_result.txt` stores the SCARA Webots smoke output.
- `run_model_stress_tests` runs seeded FK/IK regression and random stress cases for the installed Robodyno model library.
- If GitHub shows broken text again, make sure the file stays saved as UTF-8.
