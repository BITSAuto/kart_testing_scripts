#!/usr/bin/env python3
"""p14 vehicle measurements form: geometry, mass and sensor mounts for the digital twin.

Asks one question at a time (empty answer = skip / keep the previous value) and saves
data/calibration/vehicle.json plus a copy in the run folder. Run it again any time to fill in
or correct values; earlier answers are offered as defaults. Nothing is sent to the kart.

Frame for sensor positions: origin at the centre of the REAR AXLE, at axle height;
x forward, y left, z up, metres. Angles in degrees: roll (about x), pitch (about y, nose down
positive), yaw (about z, left positive). For a camera, give the direction it looks.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kart.harness import run  # noqa: E402

SENSORS = [("realsense", "RealSense camera (left lens / camera centre)"),
           ("mid360", "Livox Mid-360 (centre of the bottom mounting face)"),
           ("vectornav", "VectorNav IMU unit"),
           ("gnss_antenna_a", "VectorNav GNSS antenna A"),
           ("gnss_antenna_b", "VectorNav GNSS antenna B (if dual antenna)")]

FIELDS = [
    ("make_model", "text", "Kart make and model (or 'custom')"),
    ("wheelbase_m", "number", "Wheelbase, front axle centre to rear axle centre (m)"),
    ("front_track_m", "number", "Front track, centre of left tyre to centre of right tyre (m)"),
    ("rear_track_m", "number", "Rear track (m)"),
    ("tyre_size", "text", "Tyre size as printed on the sidewall (e.g. 18x8.50-8)"),
    ("front_wheel_radius_m", "number", "Front wheel loaded radius: ground to axle centre (m)"),
    ("rear_wheel_radius_m", "number", "Rear wheel loaded radius (m)"),
    ("tyre_width_m", "number", "Tyre width (m)"),
    ("length_m", "number", "Overall length (m)"),
    ("width_m", "number", "Overall width (m)"),
    ("height_m", "number", "Overall height incl. sensors (m)"),
    ("ground_clearance_m", "number", "Ground clearance, lowest point (m)"),
    ("front_overhang_m", "number", "Front bumper ahead of the front axle (m)"),
    ("rear_overhang_m", "number", "Rear bumper behind the rear axle (m)"),
    ("mass_total_kg", "number", "Total mass as tested (with batteries, sensors, riders) (kg)"),
    ("mass_front_axle_kg", "number", "Mass on the front axle (scales under front wheels) (kg)"),
    ("mass_rear_axle_kg", "number", "Mass on the rear axle (kg)"),
    ("cg_height_m", "number", "Estimated centre-of-mass height above ground (m)"),
    ("riders", "number", "Riders on board during the tests"),
    ("driven_wheels", "text", "Driven wheels: rear / front / all"),
    ("differential", "text", "Differential: open / solid axle / none"),
    ("suspension", "text", "Suspension (none / front only / all), describe briefly"),
    ("steering_wheel_lock_to_lock_turns", "number",
     "Steering-wheel turns lock to lock (count by hand)"),
    ("motor_notes", "text", "Drive motor / controller (rating, voltage), if known"),
    ("brake_notes", "text", "Brake type and how I=1 actuates it"),
]


def add_args(ap):
    ap.add_argument("--skip-sensors", action="store_true")
    ap.add_argument("--only-sensors", action="store_true")


def body(ctx):
    cal, ch = ctx.cal, ctx.ch
    prev = cal.get("vehicle") or {}
    out = {k: v for k, v in prev.items() if k not in ("saved_at", "source_run")}
    ctx.log("Empty answer = skip (keeps the previous value). Frame for positions: rear-axle "
            "centre, x forward, y left, z up (m).", "step")

    def q(key, kind, text, store):
        old = store.get(key)
        ans = ch.ask(text + (f" [now: {old}]" if old not in (None, "") else ""), kind,
                     allow_empty=True, default="" if kind != "number" else None)
        if ans not in (None, ""):
            store[key] = ans
            ctx.rec.event("answer", key=key, value=ans)

    if not ctx.args.only_sensors:
        ctx.set_phase("vehicle")
        for key, kind, text in FIELDS:
            q(key, kind, text, out)
    if not ctx.args.skip_sensors:
        sensors = out.setdefault("sensors", {})
        for sid, name in SENSORS:
            ctx.set_phase(f"sensor: {sid}")
            s = sensors.setdefault(sid, {})
            q("model", "text", f"{name}: model (empty = not fitted / skip)", s)
            if not s.get("model") and sid != "vectornav":
                continue
            for axis in ("x", "y", "z"):
                q(f"{axis}_m", "number", f"{name}: {axis} (m)", s)
            if not sid.startswith("gnss"):
                for ang in ("roll", "pitch", "yaw"):
                    q(f"{ang}_deg", "number", f"{name}: {ang} (deg)", s)
    if out.get("mass_front_axle_kg") and out.get("mass_rear_axle_kg") and out.get("wheelbase_m"):
        mf, mr = out["mass_front_axle_kg"], out["mass_rear_axle_kg"]
        out["cg_x_from_rear_axle_m"] = round(out["wheelbase_m"] * mf / (mf + mr), 4)
        ctx.log(f"centre of mass {out['cg_x_from_rear_axle_m']} m ahead of the rear axle", "ok")
    cal.save("vehicle", out, ctx.rec.dir)
    ctx.rec.write_json("measurements.json", out)
    missing = [t for k, kind, t in FIELDS if k not in out and kind == "number"]
    ctx.log(f"saved; {len(missing)} numeric fields still empty", "ok" if not missing else "warn")
    return {"filled": len(out), "missing_numeric": len(missing)}


if __name__ == "__main__":
    run("14_vehicle_measurements", "Geometry, mass and sensor mounts for the twin", body,
        add_args, motion=False, open_link=False, send_neutral=False)
