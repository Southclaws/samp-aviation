#include <a_samp>

#define YSI_NO_MODE_CACHE
#define YSI_NO_HEAP_MALLOC

#include <YSI_Coding\y_timers>
#include <sscanf2>
#include <zcmd>
#include <rotations.inc>
#include <mathutil>

#include "utils.inc"
#include "aviation.inc"
#include "get-angular-velocity.inc"
#include "course-heading.inc"
#include "ils-approach.inc"
#include "pfd.inc"


const Float:AIRSTRIP_X = -1127.5;
const Float:AIRSTRIP_Y = 365.5;
const Float:AIRSTRIP_Z = 13.2;
const Float:AIRSTRIP_H = 315.0;

const Float:AIRCRAFT_PROCEDURE_RADIUS = 500.0;

main() {}

static PlayerText:MessageTextdraw[MAX_PLAYERS];

// Pitch capture window is the range on either side of the target pitch at which
// the aircraft starts leveling out to maintain the pitch.
static const Float:PITCH_CAPTURE_WINDOW = 250.0;

// Heading capture window is the range on either side of the target heading at
// which the aircraft starts leveling out to maintain the heading.
static const Float:HEADING_CAPTURE_WINDOW = 15.0;

// Operation limits for most aircraft.
stock static const Float:MAX_PITCH = 15.0;
stock static const Float:MAX_ROLL = 15.0;
stock static const Float:MAX_YAW = 15.0;

static bool:AP = false;
static bool:ILS = false;
static Float:TargetAlt = 500.0;
static Float:TargetHeading = 315.0;

static Float:TurbulenceMultiplier = 1.0;

static PlayerText:PFDInterface_DistBearing[MAX_PLAYERS];
static PlayerText:PFDInterface_Altitude[MAX_PLAYERS];
static PlayerText:PFDInterface_Glideslope[MAX_PLAYERS];
static PlayerText:PFDInterface_Autopilot[MAX_PLAYERS];
static PlayerText:PFDInterface_LocHea[MAX_PLAYERS];
static PlayerText:PFDInterface_FlpLeg[MAX_PLAYERS];

forward ApplyTurbulence(playerid);
forward AngularVelocityTick(playerid);

public OnGameModeInit() {
    AddPlayerClass(61, -1657.4613, -164.8150, 13.9812, -45.0, 0, 0, 0, 0, 0, 0);

    AddStaticVehicle(400, -1660.6632, -169.1763, 13.9812, -45.0, 0, 0);
    AddStaticVehicle(593, -1650.6632, -159.1763, 14.9812, -45.0, 0, 0);
    AddStaticVehicle(460, -1640.6632, -159.1763, 14.9812, -45.0, 0, 0);
    AddStaticVehicle(476, -1620.6632, -149.1763, 14.9812, -45.0, 0, 0);
    AddStaticVehicle(511, -1600.6632, -139.1763, 14.9812, -45.0, 0, 0);
    AddStaticVehicle(512, -1580.6632, -129.1763, 14.9812, -45.0, 0, 0);
    AddStaticVehicle(513, -1560.6632, -119.1763, 14.9812, -45.0, 0, 0);
    AddStaticVehicle(519, -1540.6632, -109.1763, 14.9812, -45.0, 0, 0);
    AddStaticVehicle(520, -1520.6632, -99.1763, 14.9812, -45.0, 0, 0);
    AddStaticVehicle(553, -1500.6632, -89.1763, 14.9812, -45.0, 0, 0);
    AddStaticVehicle(577, -1450.6632, -79.1763, 14.9812, -45.0, 0, 0);
    AddStaticVehicle(592, -1400.6632, -69.1763, 14.9812, -45.0, 0, 0);
}

public OnPlayerConnect(playerid) {
    MessageTextdraw[playerid] = CreatePlayerTextDraw(playerid, 560.000091, 100.0, "");
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

    CreatePFDInterface(
        playerid,
        PFDInterface_DistBearing[playerid],
        PFDInterface_Altitude[playerid],
        PFDInterface_Glideslope[playerid],
        PFDInterface_Autopilot[playerid],
        PFDInterface_LocHea[playerid],
        PFDInterface_FlpLeg[playerid]
    );

    SetTimerEx("ApplyTurbulence", 700, true, "d", playerid);
    SetTimerEx("AngularVelocityTick", 100, true, "d", playerid);
}

// Random velocity nudges in all directions. Very basic turbulence to add a bit
// of challenge to flying planes.
public ApplyTurbulence(playerid) {
    // new vehicleid = GetPlayerVehicleID(playerid);
    // if(!IsVehicleModelPlane(GetVehicleModel(vehicleid))) {
    //     return;
    // }

    // new Float:vx, Float:vy, Float:vz;
    // GetVehicleVelocity(vehicleid, vx, vy, vz);
    // SetVehicleVelocity(vehicleid,
    //     vx,
    //     vy + (frandom(0.06, -0.01) * TurbulenceMultiplier),
    //     vz + (frandom(0.01, -0.01) * TurbulenceMultiplier)
    // );
}

public OnPlayerUpdate(playerid) {
    new vehicleid = GetPlayerVehicleID(playerid);
    if(!IsValidVehicle(vehicleid)) {
        return 1;
    }

    new modelid = GetVehicleModel(vehicleid);
    if(!IsVehicleModelPlane(modelid)) {
        return 1;
    }

    new
        Float:vx,
        Float:vy,
        Float:vz;
    GetVehicleVelocity(vehicleid, vx, vy, vz);
    new Float:lvx, Float:lvy, Float:lvz;
    GetVehicleLocalVelocity(vehicleid, lvx, lvy, lvz);

    new Float:air_speed = GetAirSpeed(vx, vy, vz);

    new Float:ground_speed = GetGroundSpeed(vx, vy);

    new Float:heading;
    GetVehicleZAngle(vehicleid, heading);

    new
        Float:roll,
        Float:pitch,
        Float:yaw;
    GetVehicleLocalRotation(vehicleid, roll, pitch, yaw);

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

    new Float:distance_to_course = GetDistancePointLine(
        AIRSTRIP_X, AIRSTRIP_Y, AIRSTRIP_Z,
        floatsin(-AIRSTRIP_H, degrees) * floatcos(3.0, degrees),
        floatcos(-AIRSTRIP_H, degrees) * floatcos(3.0, degrees),
        floatsin(3.0, degrees),
        x, y, z
    );

    new Float:distance_to_waypoint = GetDistance3D(
        AIRSTRIP_X,
        AIRSTRIP_Y,
        AIRSTRIP_Z,
        x, y, z
    );

    new Float:angle_to_aircraft = localiseHeadingAngle(
        GetAbsoluteAngle(GetAngleToPoint(
            AIRSTRIP_X, AIRSTRIP_Y, x, y
        ) - AIRSTRIP_H));

    new Float:target_heading = TargetHeading;
    new Float:target_altitude = TargetAlt;

    new bool:captured_localiser = false;
    new bool:captured_glideslope = false;
    if(ILS) {
        new
            Float:beacon_x = AIRSTRIP_X,
            Float:beacon_y = AIRSTRIP_Y;
        GetXYFromAngle(beacon_x, beacon_y, AIRSTRIP_H, 1000.0);

        if(angle_to_aircraft > 30) {
            captured_localiser = false;
            ILS = false;
        } else {
            captured_localiser = true;
        }

        new Float:course_target_heading = 0.0;
        new Float:course_target_altitude = 0.0;

        if(distance_to_course > AIRCRAFT_PROCEDURE_RADIUS) {
            if(angle_to_aircraft < 0.0) {
                course_target_heading = GetAbsoluteAngle((AIRSTRIP_H - 180.0) - 40.0);
            } else {
                course_target_heading = GetAbsoluteAngle((AIRSTRIP_H - 180.0) + 40.0);
            }
        } else {
            course_target_heading = GetHeadingForWaypointCourse(
                x, y,
                AIRSTRIP_X,
                AIRSTRIP_Y,
                GetAbsoluteAngle(AIRSTRIP_H - 180.0),
                AIRCRAFT_PROCEDURE_RADIUS
            );
            course_target_altitude = 3.28084 *
                (AIRSTRIP_Z + GetAltitudeForILS(distance_to_waypoint));
            target_altitude = course_target_altitude;
            captured_glideslope = true;
        }

        target_heading = course_target_heading;
    }

    // in order to avoid any complications and additional comparison branches
    // regarding angles, the target heading is used to calculate an offset from
    // the heading. This number will always be in the range -180..180 which
    // means all comparisons are done in relative terms rather than abstract.
    new Float:target_heading_offset = localiseHeadingAngle(GetAbsoluteAngle(heading - target_heading));

    // Altitude is measured in feet.
    new Float:altitude = GetAltitudeInFeet(z);

    new Float:target_attitude_trim = GetAircraftElevatorTrim(modelid);

    new
        Float:attitude_progress,
        Float:roll_progress,
        Float:heading_progress;

    GetTargetOrientationProgress(
        altitude,
        pitch,
        roll,

        target_heading_offset,
        target_altitude,

        PITCH_CAPTURE_WINDOW,
        HEADING_CAPTURE_WINDOW,
        MAX_PITCH,
        MAX_ROLL,
        target_attitude_trim,

        attitude_progress,
        roll_progress,
        heading_progress
    );

    new
        Float:target_pitch_velocity = 0.0,
        Float:target_roll_velocity = 0.0,
        Float:target_yaw_velocity = 0.0;

    GetTargetAngularVelocities(
        attitude_progress,
        roll_progress,
        heading_progress,

        target_pitch_velocity,
        target_roll_velocity,
        target_yaw_velocity
    );

    // This is just some very basic wobble to add some extra unexpected
    // rotations into the mix. It makes the AP look a little less robotic and
    // slightly more realistic!
    new Float:pitch_turbulence = frandom(0.0005, -0.0005);
    new Float:roll_turbulence = frandom(0.0005, -0.0005);
    new Float:yaw_turbulence = frandom(0.0005, -0.0005);

    // Auto pilot
    if(AP && air_speed > 50) {
        SetVehicleLocalAngularVelocity(vehicleid,
            target_pitch_velocity + (pitch_turbulence * TurbulenceMultiplier),
            target_roll_velocity + (roll_turbulence * TurbulenceMultiplier),
            target_yaw_velocity + (yaw_turbulence * TurbulenceMultiplier));
    }

    static
        txt_dist_bearing[64],
        txt_altitude[32],
        txt_glideslope[7],
        txt_autopilot[7],
        txt_loc_hea[16],
        txt_flp_leg[] = "(N/A)";
    
    new Float:course_target_bearing = TargetHeading;
    if(ILS) {
        course_target_bearing = AIRSTRIP_H;
    }

    GeneratePFDStrings(
        distance_to_waypoint / 1852, // metres to nautical miles
        course_target_bearing,
        TargetAlt,
        AP,
        captured_glideslope,
        captured_localiser,
        AP && !ILS,

        txt_dist_bearing,
        txt_altitude,
        txt_glideslope,
        txt_autopilot,
        txt_loc_hea
    );

    PlayerTextDrawSetString(playerid, PFDInterface_DistBearing[playerid], txt_dist_bearing);
    PlayerTextDrawSetString(playerid, PFDInterface_Altitude[playerid], txt_altitude);
    PlayerTextDrawSetString(playerid, PFDInterface_Glideslope[playerid], txt_glideslope);
    PlayerTextDrawSetString(playerid, PFDInterface_Autopilot[playerid], txt_autopilot);
    PlayerTextDrawSetString(playerid, PFDInterface_LocHea[playerid], txt_loc_hea);
    PlayerTextDrawSetString(playerid, PFDInterface_FlpLeg[playerid], txt_flp_leg);

    PlayerTextDrawShow(playerid, PFDInterface_DistBearing[playerid]);
    PlayerTextDrawShow(playerid, PFDInterface_Altitude[playerid]);
    PlayerTextDrawShow(playerid, PFDInterface_Glideslope[playerid]);
    PlayerTextDrawShow(playerid, PFDInterface_Autopilot[playerid]);
    PlayerTextDrawShow(playerid, PFDInterface_LocHea[playerid]);
    PlayerTextDrawShow(playerid, PFDInterface_FlpLeg[playerid]);

    new str[1024];
    format(str, sizeof str,
        "attitude_progress: %f~n~\
        target_pitch_velocity: %f~n~\
        roll_progress: %f~n~\
        target_roll_velocity: %f~n~\
        heading_progress: %f~n~\
        target_yaw_velocity: %f~n~\
        ~n~\
        heading: %f~n~\
        target_heading: %f~n~\
        target_heading_offset: %f~n~\
        altitude %f~n~\
        target altitude: %f~n~\
        ~n~\
        distance_to_course: %f~n~\
        angle_to_aircraft: %f~n~\
        ~n~\
        air_speed: %f~n~\
        ground_speed: %f~n~\
        pitch %f~n~\
        roll %f~n~\
        yaw %f~n~\
        lvx: %f~n~\
        lvy: %f~n~\
        lvz: %f~n~\
        pitch V %f~n~\
        roll V %f~n~\
        yaw V %f~n~\
        ",
        attitude_progress,
        target_pitch_velocity,
        roll_progress,
        target_roll_velocity,
        heading_progress,
        target_yaw_velocity,

        heading,
        target_heading,
        target_heading_offset,
        altitude,
        TargetAlt,

        distance_to_course,
        angle_to_aircraft,

        air_speed,
        ground_speed,
        pitch,
        roll,
        yaw,
        lvx,
        lvy,
        lvz,
        pitch_velocity,
        roll_velocity,
        yaw_velocity
    );
    PlayerTextDrawSetString(playerid, MessageTextdraw[playerid], str);

    return 1;
}

public OnPlayerKeyStateChange(playerid, newkeys, oldkeys) {
    if(newkeys &  KEY_FIRE) {
        AP = !AP;
        new str[14];
        format(str, sizeof str, "Auto pilot: %d", AP);
        SendClientMessage(playerid, 0xFFFFFFFF, str);
    }
}

// -
// Commands
// -

CMD:ils(playerid, params[]) {
    ILS = !ILS;
    new str[14];
    format(str, sizeof str, "ILS Approach mode: %d", ILS);
    SendClientMessage(playerid, 0xFFFFFFFF, str);
    return 1;
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
    SetPlayerPos(playerid, 239.6632, 1009.1763, 300.0);
    new v = CreateVehicle(593, 239.6632, 1009.1763, 300.0, 135.0, 0, 0, 0, 0);
    PutPlayerInVehicle(playerid, v, 0);
    return 1;
}

CMD:w(playerid, params[]) {
    SetWeather(strval(params));
    return 1;
}

CMD:targetalt(playerid, params[]) {
    sscanf(params, "f", TargetAlt);
    new str[128];
    format(str, sizeof str, "Target altitude: %fft", TargetAlt);
    SendClientMessage(playerid, 0xFFFFFFFF, str);
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
    sscanf(params, "fff", ox, oy, oz);

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
    CreateVehicle(modelid, x, y, z + 1.5, 0.0, 0, 0, 0, 0);
    return 1;
}

CMD:up(playerid, params[]) {
    new Float:x, Float:y, Float:z;
    GetVehiclePos(GetPlayerVehicleID(playerid), x, y, z);
    SetVehiclePos(GetPlayerVehicleID(playerid), x, y, z + 1000);
    return 1;
}

CMD:turb(playerid, params[]) {
    return !sscanf(params, "f", TurbulenceMultiplier);
}
