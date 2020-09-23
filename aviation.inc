// Altitude is measured in feet-above-sea-level.
static const Float:METRES_TO_FEET_RATIO = 3.28084;

// 
const Float:METRES_PER_SECOND_KNOTS_RATIO = 1.94384;

static const Float:TARGET_PITCH_MULTIPLIER = 0.01;
static const Float:TARGET_ROLL_MULTIPLIER = 0.006;
static const Float:TARGET_YAW_MULTIPLIER = -0.002;

// GetTargetOrientationProgress takes all aircraft state and calculates a set of
// "progress" values which are in the range -1.0..1.0 and indicate how close to
// the target orientations the aircraft is.
//
// This also takes into account the capture windows and clamps the output values
// based on the aircraft's position in relation to the capture windows.
stock GetTargetOrientationProgress(
    Float:altitude,
    Float:pitch,
    Float:roll,

    Float:target_heading_offset,
    Float:target_altitude,

    Float:pitch_capture_window,
    Float:heading_capture_window,
    Float:max_pitch,
    Float:max_roll,
    Float:target_attitude_trim,

    &Float:attitude_progress,
    &Float:roll_progress,
    &Float:heading_progress
) {
    // for pitch adjustment, we want to produce a multiplier between -1.0 and
    // 1.0 that is applied to the pitch angle depending on how close to the
    // target altitude the plane is. When the plane is at the target altitude,
    // the multiplier is at 0.0 and the altitude is considered captured. If the
    // plane is outside the capture window, the multiplier is at either -1.0 or 1
    // 0 and the plane is pitched at the maximum attitude (up or down) to achieve
    // the target. If the plane is within the capture window, the altitude
    // multiplier moves linearly towards 0.0 to achieve a smooth transition. This
    // also enables altitude hold mode using small nudges up and down.
    //
    // This multiplier value is stored inside `altitude_progress`.

    new Float:altitude_window_low = target_altitude - pitch_capture_window;
    new Float:altitude_window_high = target_altitude + pitch_capture_window;
    new Float:altitude_progress = 0.0;
    if(altitude < target_altitude) {
        if(altitude > altitude_window_low) {
            altitude_progress = ((-1.0) / (target_altitude - altitude_window_low)) * (altitude - target_altitude);
        } else {
            altitude_progress = 1.0;
        }
    } else {
        if(altitude < altitude_window_high) {
            altitude_progress = -(((-1.0) / (target_altitude - altitude_window_high)) * (altitude - target_altitude));
        } else {
            altitude_progress = -1.0;
        }
    }

    // Now we need the target attitude of the plane (also known as pitch). This
    // value is calculated as a simple multiplication of the maximum pitch by
    // the altitude_progress value. Since it's -1..1 the pitch range is
    // -MAX..MAX with a smooth transition in between. This keeps the plane at an
    // attitude that will eventually achieve the target altitude and is
    // calculated using very similar rules to the altitude progress. There's a
    // capture window which is 10 degrees below and above the target and this
    // results in a similar linear interpolation between the window edge and
    // the target, which resides at 0.0.
    //
    // There's also an additional component to this: `target_attitude_trim`
    // which is somewhat of a game engine related adjustment but also can serve
    // as an analogy to real-life aerodynamics too. Each plane in the game has
    // some amount of lift. This lift will raise the plane when it's travelling
    // at a constant zero attitude. This is also how wings work in the real
    // world (though they are far more complicated and are not accurately
    // modelled here obviously!) as a result of this, the plane, in order to
    // maintain a constant pitch of zero when the altitude has been captured,
    // must be trimmed down somewhat so it is at a negative pitch when
    // maintaining the target altitude. You can read more about elevator trim
    // in aviation textbooks.

    new Float:target_attitude = (max_pitch * altitude_progress) - target_attitude_trim;
    new Float:attitude_window_low = target_attitude - 10.0;
    new Float:attitude_window_high = target_attitude + 10.0;
    if(pitch < target_attitude) {
        if(pitch > attitude_window_low) {
            attitude_progress = ((-1.0) / (target_attitude - attitude_window_low)) * (pitch - target_attitude);
        } else {
            attitude_progress = 1.0;
        }
    } else {
        if(pitch < attitude_window_high) {
            attitude_progress = -(((-1.0) / (target_attitude - attitude_window_high)) * (pitch - target_attitude));
        } else {
            attitude_progress = -1.0;
        }
    }

    // Heading is treated similarly to the above two values. There's a capture
    // window, a progress value from -1.0 to 1.0 which is used to calculate
    // a target orientation of some sort. The difference here is the comparisons
    // are not using absolute value comparisons but just the difference between
    // the heading and the target heading (target_heading_offset). The capture
    // windows are also calculated differently and the linear interpolation is
    // just a basic mapping from the heading capture window range to -1..1.
    // So, the result of this is: when the plane is heading towards its target,
    // the `heading_progress` value is zero. If it's outside the capture window
    // left it's 1.0 and if it's outside the capture window right, it's -1.0 and
    // all values in between are interpolated long a linear curve.

    if(target_heading_offset < 0.0) {
        if(target_heading_offset > -heading_capture_window) {
            heading_progress = target_heading_offset / heading_capture_window;
        } else {
            heading_progress = -1.0;
        }
    } else {
        if(target_heading_offset < heading_capture_window) {
            heading_progress = target_heading_offset / heading_capture_window;
        } else {
            heading_progress = 1.0;
        }
    }

    // So, in order to rotate the plane, it must be rolled. Planes make heading
    // adjustments not by using the rudder (contrary to popular belief) but they
    // roll torwards the target heading and pitch up slightly. You can learn
    // about the aerodynamics of this elsewhere. The game does a fairly okay job
    // at simulating this behaviour so most of the time we just need to roll.
    //
    // Generally, rolls are no more than 15 degrees unless you're in special
    // circumstances. So, we just need to take the `heading_progress` value from
    // above (which is -1.0 to 1.0) and multiply it by 15. If the plane needs
    // to turn right, the target roll is somewhere between 15 and 0 and left
    // turns are the same but negative. Just like above, there's a window for
    // smooth transitions which is 3 degrees.

    new Float:target_roll = (max_roll * heading_progress);
    new Float:roll_window_low = target_roll - 3.0;
    new Float:roll_window_high = target_roll + 3.0;
    if(roll < target_roll) {
        if(roll > roll_window_low) {
            roll_progress = ((-1.0) / (target_roll - roll_window_low)) * (roll - target_roll);
        } else {
            roll_progress = 1.0;
        }
    } else {
        if(roll < roll_window_high) {
            roll_progress = -(((-1.0) / (target_roll - roll_window_high)) * (roll - target_roll));
        } else {
            roll_progress = -1.0;
        }
    }

    return 1;
}

GetTargetAngularVelocities(
    Float:attitude_progress,
    Float:roll_progress,
    Float:heading_progress,
    &Float:target_pitch_velocity,
    &Float:target_roll_velocity,
    &Float:target_yaw_velocity
) {
    // And finally, all the numbers are multiplied together in order to get our
    // final numbers. These numbers are angular velocities to nudge the plane
    // in the desired direction. Each dimension of movement has some constant
    // multiplier associated with it to bring the -1.0..1.0 ranged value into
    // some space that makes sense in the context of vehicle angular velocities.
    //
    // Note: I have no idea what the actual units are for the 
    // SetVehicleAngularVelocity function.

    // Planes roll and also pitch up in order to turn properly. So the target
    // pitch also takes into account the desired roll a tiny bit by adding some
    // value derived from the roll progress.
    target_pitch_velocity =
        (attitude_progress * TARGET_PITCH_MULTIPLIER) +
        floatabs(roll_progress * 0.001);

    target_roll_velocity = roll_progress * TARGET_ROLL_MULTIPLIER;

    // Yes, there is some yaw movement involved. Most planes don't actually use
    // the rudder for large turns like this but the game simulation is not
    // perfect so there's a bit of yaw movement to help it.
    target_yaw_velocity = heading_progress * TARGET_YAW_MULTIPLIER;
}

stock SetVehicleLocalAngularVelocity(vehicleid, Float:vx, Float:vy, Float:vz) {
    new
        Float:w,
        Float:x,
        Float:y,
        Float:z;
    GetVehicleRotationQuat(vehicleid, w, x, y, z);

    new Float:matrix[4][4];
    GetRotationMatrixFromQuat(matrix, w, x, y, z);

    new Float:wvx,
        Float:wvy,
        Float:wvz;
    MatrixRotate(matrix, vx, vy, vz, 0.0, wvx, wvy, wvz);
    return SetVehicleAngularVelocity(vehicleid, wvx, wvy, wvz);
}

stock GetVehicleRotationEuler(vehicleid, &Float:roll, &Float:pitch, &Float:yaw) {
    new
        Float:qw,
        Float:qx,
        Float:qy,
        Float:qz;
    GetVehicleRotationQuat(vehicleid, qw, qx, qy, qz);
    GetEulerFromQuat(qw, qx, qy, qz, roll, pitch, yaw);
    return 1;
}

// localiseHeadingAngle will transform a heading angle into a value that is in
// the range -180.0..180.0 in order to simplify offset calculations. For example
// if a heading angle is 270.0, determining how far to turn in order to achieve
// a heading of 45.0 degrees is not a trivial calculation. But, calculating the
// necessary change from -90.0 degrees to 45.0 is much simpler.
stock Float:localiseHeadingAngle(Float:heading) {
    return heading > 180.0 ? heading - 360.0 : heading;
}

stock GetVehicleLocalRotation(vehicleid, Float:roll, Float:pitch, Float:yaw) {
    new
        Float:qw,
        Float:qx,
        Float:qy,
        Float:qz;
    GetVehicleRotationQuat(vehicleid, qw, qx, qy, qz);
    GetEulerFromQuat(qw, qx, qy, qz, pitch, roll, yaw);

    return 1;
}

stock Float:GetAirSpeed(Float:x, Float:y, Float:z) {
    return VectorSize(x, y, z) * 50 * METRES_PER_SECOND_KNOTS_RATIO;
}

stock Float:GetGroundSpeed(Float:x, Float:y) {
    return VectorSize(x, y, 0.0) * 50 * METRES_PER_SECOND_KNOTS_RATIO;
}

stock Float:GetAltitudeInFeet(Float:z) {
    return z * METRES_TO_FEET_RATIO;
}
