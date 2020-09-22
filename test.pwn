#include <a_samp>
#include <sscanf2>
#include <zcmd>
#include <rotations.inc>
#include <mathutil>

#include "aviation.inc"


main() {}

static PlayerText:MessageTextdraw[MAX_PLAYERS];

// Altitude is measured in feet-above-sea-level.
static const Float:METRES_TO_FEET_RATIO = 3.28084;

// Velocity is metres per 50th of a second. So to calculate the vertical speed
// first multiply by 50 then 196.85
static const Float:FPM_RATIO = 196.85;

// Pitch capture window is the range on either side of the target pitch at which
// the aircraft starts leveling out to maintain the pitch.
static const PITCH_CAPTURE_WINDOW = 250;

// Heading capture window is the range on either side of the target heading at
// which the aircraft starts leveling out to maintain the heading.
static const Float:HEADING_CAPTURE_WINDOW = 15.0;

// Operation limits for most aircraft.
static const Float:MAX_PITCH = 15.0;
static const Float:MAX_ROLL = 15.0;
static const Float:MAX_YAW = 15.0;

static bool:AP = false;
static TargetAlt = 500;
static Float:TargetVerticalSpeed = 1000.0;
static Float:TargetHeading = 315.0;
static Float:target_pitch_multiplier = 0.01;
static Float:target_roll_multiplier = 0.006;
static Float:target_yaw_multiplier = -0.002;

// Pitch trim is an additional offset applied to the pitch angle in order to
// keep the aircraft on a level course. It's different for every aircraft as
// each aircraft has a different lift profile based on its size, speed, etc.
// TODO: per-aircraft values.
static Float:target_attitude_trim = 3.35; // hydra: 1.5, rustler: 3.4

forward ApplyTurbulence(playerid);
forward AngularVelocityTick(playerid);

public OnGameModeInit() {
    AddPlayerClass(61, -1657.4613, -164.8150, 13.9812, -45.0, 0, 0, 0, 0, 0, 0);

    AddStaticVehicle(400, -1660.6632, -169.1763, 13.9812, -45.0, 0, 0);
    AddStaticVehicle(593, -1650.6632, -159.1763, 13.9812, -45.0, 0, 0);
    AddStaticVehicle(460, -1640.6632, -159.1763, 13.9812, -45.0, 0, 0);
    AddStaticVehicle(476, -1620.6632, -149.1763, 13.9812, -45.0, 0, 0);
    AddStaticVehicle(511, -1600.6632, -139.1763, 13.9812, -45.0, 0, 0);
    AddStaticVehicle(512, -1580.6632, -129.1763, 13.9812, -45.0, 0, 0);
    AddStaticVehicle(513, -1560.6632, -119.1763, 13.9812, -45.0, 0, 0);
    AddStaticVehicle(519, -1540.6632, -109.1763, 13.9812, -45.0, 0, 0);
    AddStaticVehicle(520, -1520.6632, -99.1763, 13.9812, -45.0, 0, 0);
    AddStaticVehicle(553, -1500.6632, -89.1763, 13.9812, -45.0, 0, 0);
    AddStaticVehicle(577, -1450.6632, -79.1763, 13.9812, -45.0, 0, 0);
    AddStaticVehicle(592, -1400.6632, -69.1763, 13.9812, -45.0, 0, 0);
}

public OnPlayerConnect(playerid) {
    MessageTextdraw[playerid] = CreatePlayerTextDraw(playerid, 560.000091, 250.947723, "");
    PlayerTextDrawLetterSize(playerid, MessageTextdraw[playerid], 0.2, 1.0);
    PlayerTextDrawAlignment(playerid, MessageTextdraw[playerid], 3);
    PlayerTextDrawColor(playerid, MessageTextdraw[playerid], -1);
    PlayerTextDrawSetShadow(playerid, MessageTextdraw[playerid], 0);
    PlayerTextDrawSetOutline(playerid, MessageTextdraw[playerid], 1);
    PlayerTextDrawBackgroundColor(playerid, MessageTextdraw[playerid], 255);
    PlayerTextDrawFont(playerid, MessageTextdraw[playerid], 1);
    PlayerTextDrawSetProportional(playerid, MessageTextdraw[playerid], 1);
    PlayerTextDrawSetShadow(playerid, MessageTextdraw[playerid], 0);
    PlayerTextDrawShow(playerid, MessageTextdraw[playerid]);

    SetTimerEx("ApplyTurbulence", 1000, true, "d", playerid);
    SetTimerEx("AngularVelocityTick", 1000, true, "d", playerid);
}

// Random velocity nudges in all directions. Very basic turbulence to add a bit
// of challenge to flying planes.
public ApplyTurbulence(playerid) {
    new vehicleid = GetPlayerVehicleID(playerid);

    if(!IsValidVehicle(vehicleid)) {
        return;
    }

    new Float:vx, Float:vy, Float:vz;
    GetVehicleVelocity(vehicleid, vx, vy, vz);
    SetVehicleVelocity(vehicleid,
        vx + frandom(0.02, -0.02),
        vy + frandom(0.02, -0.02),
        vz + frandom(0.02, -0.02));
}

public OnPlayerUpdate(playerid) {
    new vehicleid = GetPlayerVehicleID(playerid);

    if(!IsValidVehicle(vehicleid)) {
        return 1;
    }

    new
        Float:qw,
        Float:qx,
        Float:qy,
        Float:qz,
        Float:roll,
        Float:pitch,
        Float:yaw,
        Float:heading;
    GetVehicleRotationQuat(vehicleid, qw, qx, qy, qz);
    GetVehicleZAngle(vehicleid, heading);
    GetEulerFromQuat(qw, qx, qy, qz, pitch, roll, yaw);

    // adjust heading angles so they are in the same range (-180..180)
    heading = localiseHeadingAngle(heading);
    new Float:target_heading = localiseHeadingAngle(TargetHeading);

    new
        Float:pitch_velocity,
        Float:roll_velocity,
        Float:yaw_velocity;
    GetVehicleLocalAngularVelocity(vehicleid, pitch_velocity, roll_velocity, yaw_velocity);

    new
        Float:x,
        Float:y,
        Float:z;
    GetVehiclePos(vehicleid, x, y, z);

    new Float:altitude = z * METRES_TO_FEET_RATIO; 

    new Float:vx, Float:vy, Float:vz;
    GetVehicleVelocity(vehicleid, vx, vy, vz);
    new Float:air_speed = VectorSize(vx, vy, vz);
    new Float:ground_speed = VectorSize(vx, vy, 0.0);

    new Float:vertical_speed = vz * 50 * FPM_RATIO;
    new Float:vertical_speed_window_low = TargetVerticalSpeed - 250.0;
    new Float:vertical_speed_window_high = TargetVerticalSpeed + 250.0;
    new Float:vertical_speed_progress = 0.0;
    if(vertical_speed < TargetVerticalSpeed) {
        if(vertical_speed > vertical_speed_window_low) {
            vertical_speed_progress = ((-1.0) / (TargetVerticalSpeed - vertical_speed_window_low)) * (vertical_speed - TargetVerticalSpeed);
        } else {
            vertical_speed_progress = 1.0;
        }
    } else {
        if(vertical_speed < vertical_speed_window_high) {
            vertical_speed_progress = -(((-1.0) / (TargetVerticalSpeed - vertical_speed_window_high)) * (vertical_speed - TargetVerticalSpeed));
        } else {
            vertical_speed_progress = -1.0;
        }
    }

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

    new altitude_window_low = TargetAlt - PITCH_CAPTURE_WINDOW;
    new altitude_window_high = TargetAlt + PITCH_CAPTURE_WINDOW;
    new Float:altitude_progress = 0.0;
    if(altitude < TargetAlt) {
        if(altitude > altitude_window_low) {
            altitude_progress = ((-1.0) / (TargetAlt - altitude_window_low)) * (altitude - TargetAlt);
        } else {
            altitude_progress = 1.0;
        }
    } else {
        if(altitude < altitude_window_high) {
            altitude_progress = -(((-1.0) / (TargetAlt - altitude_window_high)) * (altitude - TargetAlt));
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

    new Float:target_attitude = (MAX_PITCH * altitude_progress) - target_attitude_trim;
    // I did attempt vertical speed mode using the following formula:
    // but it oscilates around the target altitude a lot for some reason.
    // new Float:target_attitude = (MAX_PITCH * vertical_speed_progress) - target_attitude_trim;
    new Float:attitude_window_low = target_attitude - 10.0;
    new Float:attitude_window_high = target_attitude + 10.0;
    new Float:attitude_progress = 0.0;
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
    // a target orientation of some sort. There is a slight difference here and
    // that is that the values are inverted. This is because, for some odd
    // reason, the game engine treats east as 270.0 and west as 90.0 degrees.
    // Aside from that, the linear interpolation here is the exact same as the
    // above values. When the plane is heading towards its target, the
    // `heading_progress` value is zero. If it's outside the capture window left
    // it's 1.0 and if it's outside the capture window right, it's -1.0 and all
    // values in between are interpolated long a linear curve.

    new Float:heading_window_low = target_heading - HEADING_CAPTURE_WINDOW;
    new Float:heading_window_high = target_heading + HEADING_CAPTURE_WINDOW;
    new Float:heading_progress = 0.0;
    if(heading < target_heading) {
        if(heading > heading_window_low) {
            heading_progress = -(((-1.0) / (target_heading - heading_window_low)) * (heading - target_heading));
        } else {
            heading_progress = -1.0;
        }
    } else {
        if(heading < heading_window_high) {
            heading_progress = (((-1.0) / (target_heading - heading_window_high)) * (heading - target_heading));
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

    new Float:target_roll = (MAX_ROLL * heading_progress);
    new Float:roll_window_low = target_roll - 3.0;
    new Float:roll_window_high = target_roll + 3.0;
    new Float:roll_progress = 0.0;
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
    new Float:target_pitch_velocity =
        (attitude_progress * target_pitch_multiplier) +
        floatabs(roll_progress * 0.001);

    new Float:target_roll_velocity = roll_progress * target_roll_multiplier;

    // Yes, there is some yaw movement involved. Most planes don't actually use
    // the rudder for large turns like this but the game simulation is not
    // perfect so there's a bit of yaw movement to help it.
    new Float:target_yaw_velocity = heading_progress * target_yaw_multiplier;

    // This is just some very basic wobble to add some extra unexpected
    // rotations into the mix. It makes the AP look a little less robotic and
    // slightly more realistic!
    new Float:pitch_turbulence_multiplier = frandom(0.0005, -0.0005);
    new Float:roll_turbulence_multiplier = frandom(0.0005, -0.0005);
    new Float:yaw_turbulence_multiplier = frandom(0.0005, -0.0005);

    // Auto pilot
    if(AP) {
        SetVehicleLocalAngularVelocity(vehicleid,
            target_pitch_velocity + pitch_turbulence_multiplier,
            target_roll_velocity + roll_turbulence_multiplier,
            target_yaw_velocity + yaw_turbulence_multiplier);
    }

    new str[680];
    format(str, sizeof str,
        "altitude_progress: %f~n~\
        target_attitude: %f~n~\
        attitude_progress: %f~n~\
        target_pitch_velocity: %f~n~\
        \
        vertical_speed_progress: %f~n~\
        \
        heading_progress: %f~n~\
        target_roll: %f~n~\
        roll_progress: %f~n~\
        target_roll_velocity: %f~n~\
        \
        air_speed: %f~n~\
        ground_speed: %f~n~\
        \
        Pitch %f Roll %f Yaw %f~n~\
        \
        Pitch V %f Roll V %f Yaw V %f~n~\
        \
        Heading: %f~n~ Target: %f~n~\
        Alt %f Target: %d~n~\
        VS %f Target: %f~n~\
        ",
        altitude_progress,
        target_attitude,
        attitude_progress,
        target_pitch_velocity,

        vertical_speed_progress,

        heading_progress,
        target_roll,
        roll_progress,
        target_roll_velocity,

        air_speed,
        ground_speed,

        pitch, roll, yaw,

        pitch_velocity, roll_velocity, yaw_velocity,

        heading, target_heading,
        altitude, TargetAlt,
        vertical_speed, TargetVerticalSpeed);
    PlayerTextDrawSetString(playerid, MessageTextdraw[playerid], str);

    return 1;
}

// localiseHeadingAngle will transform a heading angle into a value that is in
// the range -180.0..180.0 in order to simplify offset calculations. For example
// if a heading angle is 270.0, determining how far to turn in order to achieve
// a heading of 45.0 degrees is not a trivial calculation. But, calculating the
// necessary change from -90.0 degrees to 45.0 is much simpler.
Float:localiseHeadingAngle(Float:heading) {
    return heading > 180.0 ? heading - 360.0 : heading;
}

// -
// Angular velocity is not quite as accurate as it could be. Needs some tuning.
// -

static
    Float:PreviousRoll[MAX_VEHICLES],
    Float:PreviousPitch[MAX_VEHICLES],
    Float:PreviousYaw[MAX_VEHICLES],
    Float:RollVelocity[MAX_VEHICLES],
    Float:PitchVelocity[MAX_VEHICLES],
    Float:YawVelocity[MAX_VEHICLES];

forward AngularVelocityTick(playerid);
public AngularVelocityTick(playerid) {
    new vehicleid = GetPlayerVehicleID(playerid);

    if(!IsValidVehicle(vehicleid)) {
        return 1;
    }

    new
        Float:current_roll,
        Float:current_pitch,
        Float:current_yaw;
    GetVehicleRotationEuler(vehicleid, current_roll, current_pitch, current_yaw);

    RollVelocity[vehicleid] = (current_roll - PreviousRoll[vehicleid]) * 0.05;
    PitchVelocity[vehicleid] = (current_pitch - PreviousPitch[vehicleid]) * 0.05;
    YawVelocity[vehicleid] = (current_yaw - PreviousYaw[vehicleid]) * 0.05;

    PreviousRoll[vehicleid] = current_roll;
    PreviousPitch[vehicleid] = current_pitch;
    PreviousYaw[vehicleid] = current_yaw;

    return 1;
}

stock GetVehicleLocalAngularVelocity(vehicleid, &Float:vx, &Float:vy, &Float:vz) {
    vx = RollVelocity[vehicleid];
    vy = PitchVelocity[vehicleid];
    vz = YawVelocity[vehicleid];
}

public OnPlayerKeyStateChange(playerid, newkeys, oldkeys) {
    if(newkeys &  KEY_FIRE) {
        AP = !AP;
        new str[128];
        format(str, sizeof str, "Auto pilot: %d", AP);
        SendClientMessage(playerid, 0xFFFFFFFF, str);
    }
}

// -
// Commands
// -

CMD:althold(playerid, params[]) {
    new target = strval(params[10]);

    new str[128];
    format(str, sizeof str, "Target altitude: %dft", target);
    SendClientMessage(playerid, 0xFFFFFFFF, str);

    TargetAlt = target;
}

CMD:vs(playerid, params[]) {
    new Float:target = floatstr(params[4]);

    new str[128];
    format(str, sizeof str, "Target vertical speed: %fft/s", target);
    SendClientMessage(playerid, 0xFFFFFFFF, str);

    TargetVerticalSpeed = target;
}

CMD:grav(playerid, params[]) {
    SetGravity(0.0);
    return 1;
}

CMD:grav1(playerid, params[]) {
    SetGravity(0.008);
    return 1;
}

CMD:p(playerid, params[]) {
    SetPlayerPos(playerid, -1650.6632, -159.1763, 200.0);
    new v = CreateVehicle(593, -1650.6632, -159.1763, 200.0, -45.0, 0, 0, 0, 0);
    PutPlayerInVehicle(playerid, v, 0);
    return 1;
}

CMD:w(playerid, params[]) {
    SetWeather(strval(params));
    return 1;
}

CMD:tpv(playerid, params[]) {
    if(sscanf(params, "f", target_pitch_multiplier)) {
        SendClientMessage(playerid, -1, "Usage: tpv [value]");
        return 1;
    }

    new str[128];
    format(str, sizeof str, "target_pitch_multiplier: %f", target_pitch_multiplier);
    SendClientMessage(playerid, 0xFFFFFFFF, str);

    return 1;
}

CMD:tat(playerid, params[]) {
    sscanf(params, "f", target_attitude_trim);
    return 1;
}

CMD:targetalt(playerid, params[]) {
    sscanf(params, "d", TargetAlt);
    return 1;
}

CMD:targethea(playerid, params[]) {
    sscanf(params, "f", TargetHeading);
    return 1;
}

CMD:fpv(playerid, params[]) {
    new vehicleid = GetPlayerVehicleID(playerid);
    if(!IsValidVehicle(vehicleid)) {
        return 1;
    }

    new Float:ox, Float:oy, Float:oz;
    sscanf(params, "fff", oz, oy, oz);

    // someone needs to write a vscode macro/snippet for this:
    new Float:x, Float:y, Float:z;
    GetPlayerPos(playerid, x, y, z);

    new oid = CreateObject(19300, x, y, z,0.0, 0.0, 0.0, 0.0);
    AttachObjectToVehicle(oid, vehicleid, oz, oy, oz, 0.0, 0.0, 0.0);
    AttachCameraToObject(playerid, oid);

    return 1;
}

CMD:v(playerid, params[]) {
    new modelid, Float:x, Float:y, Float:z;
    sscanf(params, "d", modelid);
    GetPlayerPos(playerid, x, y, z);
    CreateVehicle(modelid, x, y, z, 0.0, 0, 0, 0, 0);
    return 1;
}
