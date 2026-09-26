# 24 V Damiao specifications and simulation limits

2026-09-26: operator identified joint1–3 as DM-J4340P-2EC V1.1 24 V,
joint4–6 as DM-J4310-2EC V1.2 24 V. Gripper hardware revision is not newly verified.

| Axes | Rated / peak torque (Nm) | Ratio | Rated / no-load speed (rpm) | Simulation peak limit (Nm) |
| --- | --- | --- | --- | --- |
| J1–3 | 12 / 40 | 40 | 36 / 56 | 27 |
| J4–6 | 3.5 / 12.5 | 10 | 120 / 200 | 7 |

`motor_control_calibration.yaml` records corrected manufacturer specifications.
At the operator's request the simulation retains the original URDF effort limits:
27 Nm for J1–3 and 7 Nm for J4–6. The temporary per-axis override was removed.
Controller torque clamp, motor ctrlrange/forcerange and joint actuatorfrcrange
follow that baseline. Manufacturer peak torque is descriptive, not an override.
No SDK, POS_VEL setting or hardware register was changed. Protocol mapping ranges
are not the same as physical protection limits. These simulation caps do not
model peak duration, thermal behavior or speed-dependent torque capability.

## Manufacturer sources

- [4340P V1.1 manufacturer repository](https://github.com/dmBots/DM-J4340P-2EC/tree/98a29a9df67c83512795f8bb9ca2bdd2423f66dd/manual)
- [4310 V1.2 manufacturer repository](https://github.com/dmBots/DM-J4310-2EC/tree/2da08d236aa06743f85cde409811c005fbae0d5c/manual)

Both English User Manuals V1.4 dated 2026-09-14, PDF page 20: read-only
Damp (0x0B/11, viscous damping), Inertia (0x0C/12, rotor inertia), Gr (0x14/20).
Parameter Identification section says viscous coefficient is for reference only;
identification rotates the motor and requires no load. No identification was run.
No fixed per-model rotor inertia, joint damping or Coulomb friction value was
found in these manuals. Units and motor/output-side conventions must be confirmed
before converting register values to MuJoCo damping/armature; do not blindly copy.
Deta (31, speed loop damping coefficient) is not passive joint damping.

## SDK audit

Installed MotorBridge: 0.4.7+rebotarm.1. Current model mappings (PMAX,VMAX,TMAX):
4340P=(12.5,10,28), 4310=(12.5,30,10).
Source: build_motorbridge_fresh_feedback/source/motor_vendors/damiao/src/motor.rs.
Register catalog already supports Damp/Inertia and generic register reads.
Manufacturer PMAX/VMAX/TMAX are configurable protocol mapping ranges, used for MIT
commands and feedback decoding (also in other control modes). Mechanical peak
ratings 40/12.5 must NOT replace TMAX without matching the actual device mapping.
No SDK upgrade/rebuild justified by the specification change alone. Hardware
compatibility is still pending disabled-only readback of firmware version, Gr,
PMAX/VMAX/TMAX and, where supported, Damp/Inertia. No hardware accessed this task.

## Verification

System tests 755 passed / 15 skipped; package layering 18 passed (within focused
19 passed / 1 skipped); MuJoCo focused 5 passed. Simulation build, installed model
load, generation consistency and compileall checked. Tests cover retained URDF limits despite updated manufacturer ratings and both compiled actuator and joint force clamps.
