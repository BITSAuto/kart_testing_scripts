"""The procedures, in the order to run them. The dashboard builds its list from this."""

PROCEDURES = [
    dict(id="p00", script="p00_serial_probe.py", title="Serial probe",
         motion=False, needs=[], args="--listen 3",
         desc="Listens to the ESP32, sends one neutral frame, lists the line formats it sees and "
              "suggests the encoder regex. Add '--pulse 40' to steer briefly and find which number "
              "is the encoder (needs SPACE). Run this first on a new setup."),
    dict(id="p01", script="p01_health_check.py", title="Health check",
         motion=False, needs=[], args="",
         desc="Port, permissions, data flow, encoder parsing, calibration age, VectorNav. "
              "'--motion-check' also pulses the steering left and right."),
    dict(id="p02", script="p02_steer_command_map.py", title="Steering command map",
         motion=True, needs=[], args="--values 40,30,20,0,60,70,80,100 --pulse-s 0.4",
         desc="Short pulses at several D values: which side is left, the deadband around 50, "
              "and whether |D-50| changes the speed. Asks you to confirm LEFT by eye. "
              "Writes calibration/steer_map.json. Front wheels off the ground or kart parked."),
    dict(id="p03", script="p03_center_and_range.py", title="Centre, locks, angle table",
         motion=True, needs=["steer_map"], args="--points 7 --both-wheels",
         desc="You jog the wheels straight (sets the zero), then to each lock; then it stops at "
              "several positions and you type the measured road-wheel angle. Writes zero.json and "
              "range.json. '--zero-only' just re-zeros (do that after every ESP32 power cycle)."),
    dict(id="p04", script="p04_slew_rate.py", title="Steering rate (slew)",
         motion=True, needs=["steer_map", "zero", "range"], args="--reps 5 --span 0.8",
         desc="Sweeps lock to lock both ways and measures the steering speed in each direction "
              "and across the range. '--throttle N' repeats it while rolling."),
    dict(id="p05", script="p05_latency_overshoot.py", title="Delay and overshoot",
         motion=True, needs=["steer_map", "zero", "range", "slew"], args="--reps 10",
         desc="Random-length moves: time from command to first movement, from STOP to standstill, "
              "and how far the wheel coasts after STOP."),
    dict(id="p06", script="p06_command_drop.py", title="Dropped commands",
         motion=True, needs=["steer_map", "zero", "range", "slew", "latency"],
         args="--spacings 0.05,0.1,0.15,0.2,0.25,0.3,0.4 --reps 8",
         desc="Sends MOVE then STOP at different spacings and checks whether the STOP took effect. "
              "Gives the chance of a dropped command against spacing."),
    dict(id="p07", script="p07_backlash.py", title="Backlash",
         motion=True, needs=["steer_map", "zero", "range"], args="--positions 0,0.5,-0.5 --reps 2",
         desc="Stops at the same encoder position coming from the left and from the right; you type "
              "the road-wheel angle each time. The difference is the play."),
    dict(id="p08", script="p08_center_repeatability.py", title="Centre repeatability",
         motion=True, needs=["steer_map", "zero", "range"], args="--trials 6",
         desc="Random excursions, back to encoder zero, you type the wheel angle. Shows encoder "
              "slip/drift."),
    dict(id="p09", script="p09_throttle_speed.py", title="Throttle -> speed",
         motion=True, needs=[], args="--throttle 60,70 --then both", vn=True,
         desc="Straight runs: acceleration to steady speed, then coast and/or brake. Needs "
              "VectorNav and a clear straight. Gives the throttle for 15 km/h."),
    dict(id="p10", script="p10_lock_circles.py", title="Full-lock circles",
         motion=True, needs=["steer_map", "zero", "range"], args="--throttle 60", vn=True,
         desc="Drives circles at full lock both ways, fits the radius from VectorNav. Checks the "
              "steering angle and gives the effective wheelbase."),
    dict(id="p11", script="p11_sysid_drive.py", title="System-ID drive",
         motion=True, needs=["steer_map", "zero", "range"], args="--duration 60",
         desc="Random left/right/stop sequence, including commands closer than 0.2 s, for fitting "
              "the actuator model. Standstill by default; '--throttle N' to drive."),
    dict(id="p12", script="p12_manual_drive_record.py", title="Manual drive (record)",
         motion=True, needs=["steer_map"], args="",
         desc="You drive from the keyboard (A/D steer, W/S throttle on/off, B brake, ENTER ends) "
              "while everything is recorded."),
    dict(id="p13", script="p13_step_steer.py", title="Step steer at speed",
         motion=True, needs=["steer_map", "zero", "range"], args="--fractions 0.3,0.6", vn=True,
         desc="Drives straight, steps the steering to 30/60 % lock each way and back. Yaw-rate "
              "delay, rise time and measured vs kinematic curvature: the tyre/understeer data "
              "the twin needs. Throttle defaults to the 15 km/h value from p09."),
    dict(id="p14", script="p14_vehicle_measurements.py", title="Vehicle measurements form",
         motion=False, needs=[], args="",
         desc="Type in wheelbase, tracks, tyres, mass (per axle), CoM height and sensor "
              "positions/angles. Saved to calibration/vehicle.json; rerun to fill gaps."),
]

BY_ID = {p["id"]: p for p in PROCEDURES}
