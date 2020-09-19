#include <a_samp>
#include <sscanf2>
#include <zcmd>
#include <rotations.inc>
#include <mathutil>

#include "aviation.inc"


main() {}

static PlayerText:MessageTextdraw[MAX_PLAYERS];

static bool:AP = false;
static TargetAlt = 500;
static Float:TargetVS = 250.0;
static Float:TargetHeading = 265.0;
static const Float:HEADING_CAPTURE_WINDOW = 15.0;
static Float:target_pitch_multiplier = 0.01;
static Float:target_roll_multiplier = 0.006;
static Float:target_yaw_multiplier = -0.002;
static Float:target_attitude_trim = 3.35;

static const Float:MAX_PITCH = 15.0;
static const Float:MAX_ROLL = 20.0;
static const Float:MAX_YAW = 15.0;
static const PITCH_CAPTURE_WINDOW = 250;

forward update(playerid);
forward turb(playerid);
forward AngularVelocityTick(playerid);

public OnGameModeInit() {
    AddPlayerClass(61, -1657.4613, -164.8150, 13.9812, -45.0, 0, 0, 0, 0, 0, 0);
    AddStaticVehicle(593, -1650.6632, -159.1763, 13.9812, -45.0, 0, 0);

    AddStaticVehicle(460, -1640.6632, -159.1763, 13.9812, -45.0, 0, 0);
    AddStaticVehicle(476, -1620.6632, -149.1763, 13.9812, -45.0, 0, 0);
    AddStaticVehicle(511, -1600.6632, -139.1763, 13.9812, -45.0, 0, 0);
    AddStaticVehicle(512, -1580.6632, -129.1763, 13.9812, -45.0, 0, 0);
    AddStaticVehicle(513, -1560.6632, -119.1763, 13.9812, -45.0, 0, 0);
    AddStaticVehicle(519, -1540.6632, -109.1763, 13.9812, -45.0, 0, 0);
    AddStaticVehicle(520, -1520.6632, -99.1763, 13.9812, -45.0, 0, 0);
    AddStaticVehicle(553, -1500.6632, -89.1763, 13.9812, -45.0, 0, 0);
    AddStaticVehicle(577, -1480.6632, -79.1763, 13.9812, -45.0, 0, 0);
    AddStaticVehicle(592, -1460.6632, -69.1763, 13.9812, -45.0, 0, 0);
    AddStaticVehicle(593, -1440.6632, -59.1763, 13.9812, -45.0, 0, 0);
}

public OnPlayerConnect(playerid) {
    MessageTextdraw[playerid] = CreatePlayerTextDraw(playerid, 560.000091, 250.947723, "");
    PlayerTextDrawLetterSize(playerid, MessageTextdraw[playerid], 0.2, 1.0);
    PlayerTextDrawAlignment(playerid, MessageTextdraw[playerid], 2);
    PlayerTextDrawColor(playerid, MessageTextdraw[playerid], -1);
    PlayerTextDrawSetShadow(playerid, MessageTextdraw[playerid], 0);
    PlayerTextDrawSetOutline(playerid, MessageTextdraw[playerid], 1);
    PlayerTextDrawBackgroundColor(playerid, MessageTextdraw[playerid], 255);
    PlayerTextDrawFont(playerid, MessageTextdraw[playerid], 1);
    PlayerTextDrawSetProportional(playerid, MessageTextdraw[playerid], 1);
    PlayerTextDrawSetShadow(playerid, MessageTextdraw[playerid], 0);
    PlayerTextDrawShow(playerid, MessageTextdraw[playerid]);

    SetTimerEx("update", 100, true, "d", playerid);
    SetTimerEx("turb", 1000, true, "d", playerid);
    SetTimerEx("AngularVelocityTick", 100, true, "d", playerid);
}

public turb(playerid) {
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
    if(heading > 180.0) {
        heading -= 360.0;
    }
    if(TargetHeading > 180.0) {
        TargetHeading -= 360.0;
    }

    new
        Float:pitch_velocity,
        Float:roll_velocity,
        Float:yaw_velocity;
    GetVehicleLocalAngularVelocity(vehicleid, pitch_velocity, roll_velocity, yaw_velocity);

    new Float:x, Float:y, Float:z;
    GetVehiclePos(vehicleid, x, y, z);

    new Float:altitude = z * 3.28084; // altitude is measured in feet-above-sea-level.

    // ground speed
    new Float:vx, Float:vy, Float:vz;
    GetVehicleVelocity(vehicleid, vx, vy, vz);
    new Float:air_speed = VectorSize(vx, vy, vz);
    new Float:ground_speed = VectorSize(vx, vy, 0.0);

    new Float:vertical_speed = vz * 1968.5; // velocity is km/s (?) so fpm conversion is one decimal place up from m/s

    // for pitch adjustment, we want to produce a multiplier between 1.0 and 0.0
    // that is applied to the pitch angle depending on how close to the target
    // altitude the plane is. When the plane is at the target altitude, the
    // multiplier is at 0.0 and the altitude is considered captured. If the
    // plane is outside the capture window, the multiplier is at 1.0 and the
    // plane is pitched at the maximum attitude to achieve the target. If the
    // plane is within the capture window, the altitude multiplier moves along
    // some curve towards 0.0 to achieve a smooth transition. This also enables
    // altitude hold mode using small nudges up and down.

    // if the plane is below our target altitude and within the capture window
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

    new Float:target_attitude = (MAX_PITCH * altitude_progress) - target_attitude_trim;
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

    new Float:heading_window_low = TargetHeading - HEADING_CAPTURE_WINDOW;
    new Float:heading_window_high = TargetHeading + HEADING_CAPTURE_WINDOW;
    new Float:heading_progress = 0.0;
    if(heading < TargetHeading) {
        if(heading > heading_window_low) {
            heading_progress = -(((-1.0) / (TargetHeading - heading_window_low)) * (heading - TargetHeading));
        } else {
            heading_progress = -1.0;
        }
    } else {
        if(heading < heading_window_high) {
            heading_progress = (((-1.0) / (TargetHeading - heading_window_high)) * (heading - TargetHeading));
        } else {
            heading_progress = 1.0;
        }
    }

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

    new Float:target_pitch_velocity = attitude_progress * target_pitch_multiplier; // + some roll manipulation for faster turns
    new Float:target_roll_velocity = roll_progress * target_roll_multiplier;
    new Float:target_yaw_velocity = heading_progress * target_yaw_multiplier;

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
        heading_progress: %f~n~\
        target_roll: %f~n~\
        roll_progress: %f~n~\
        target_roll_velocity: %f~n~\
        \
        air_speed: %f~n~\
        ground_speed: %f~n~\
        \
        Pitch %f~n~\
        Roll %f~n~\
        Yaw %f~n~\
        \
        Pitch V %f~n~\
        Roll V %f~n~\
        Yaw V %f~n~\
        \
        Heading: %f~n~ Target: %f~n~\
        Alt %f Target: %d~n~\
        VS %f Target: %f~n~\
        ",
        altitude_progress,
        target_attitude,
        attitude_progress,
        target_pitch_velocity,

        heading_progress,
        target_roll,
        roll_progress,
        target_roll_velocity,

        air_speed,
        ground_speed,

        pitch, roll, yaw,

        pitch_velocity, roll_velocity, yaw_velocity,

        heading, TargetHeading,
        altitude, TargetAlt,
        vertical_speed, TargetVS);
    PlayerTextDrawSetString(playerid, MessageTextdraw[playerid], str);

    return 1;
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

    TargetVS = target;
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
