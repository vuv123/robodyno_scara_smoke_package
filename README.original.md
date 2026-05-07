# Scara smoke package

## What this verifies

- `robodyno_damn` works inside a Webots SCARA robot world.
- `SliderModule(webots, 0x10)` constructs and commands the lead-screw abstraction.
- `Motor(webots, 0x11..0x13)` constructs without explicit `type_`.
- Webots legacy sensor aliases resolve:
  - `ROBODYNO_PRO_P44 -> Model.ROBODYNO_PRO_01B`
  - `ROBODYNO_PRO_P12 -> Model.ROBODYNO_PLUS_P12`
- Joint/slider positions move toward issued targets.

## Localized world

`ScaraLocalSmoke.wbt` is derived from upstream `ScaraRobot.wbt` with remote Cyberbotics background/floor EXTERNPROTO removed, so the test is not blocked by network fetches.

## Command used

```powershell
$env:WEBOTS_HOME='D:\webotmcp\webots\Webots'
$env:PYTHONPATH='D:\webots_refactor_brief\robodyno_damn\src'
& 'D:\webotmcp\webots\Webots\msys64\mingw64\bin\webots.exe' --batch --mode=fast --stdout --stderr --minimize 'D:\webots_refactor_brief\smoke_webots_models\worlds\ScaraLocalSmoke.wbt'
```

The controller writes:

```text
D:\webots_refactor_brief\scara_smoke_result.txt
```

## Observed pass evidence

See `scara_smoke_result.txt`. Expected decisive lines:

```text
SCARA_SMOKE: slider constructed motor_type=Model.ROBODYNO_PRO_01B
SCARA_SMOKE: motor 0x13 constructed type=Model.ROBODYNO_PLUS_P12
SCARA_SMOKE: actuators enabled
SCARA_SMOKE: done
```
