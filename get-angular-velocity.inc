// -
// Angular velocity is not quite as accurate as it could be. Needs some tuning.
// -

#include <YSI_Coding\y_timers>

static
    Float:PreviousRoll[MAX_VEHICLES],
    Float:PreviousPitch[MAX_VEHICLES],
    Float:PreviousYaw[MAX_VEHICLES],
    Float:RollVelocity[MAX_VEHICLES],
    Float:PitchVelocity[MAX_VEHICLES],
    Float:YawVelocity[MAX_VEHICLES];

ptask calculateAngularVelocity[100](playerid) {
    new vehicleid = GetPlayerVehicleID(playerid);

    if(!IsValidVehicle(vehicleid)) {
        return 1;
    }

    new
        Float:current_roll,
        Float:current_pitch,
        Float:current_yaw;
    GetVehicleRotationEuler(vehicleid, current_roll, current_pitch, current_yaw);

    RollVelocity[vehicleid] = (current_roll - PreviousRoll[vehicleid]) * 10.0;
    PitchVelocity[vehicleid] = (current_pitch - PreviousPitch[vehicleid]) * 10.0;
    YawVelocity[vehicleid] = (current_yaw - PreviousYaw[vehicleid]) * 10.0;

    PreviousRoll[vehicleid] = current_roll;
    PreviousPitch[vehicleid] = current_pitch;
    PreviousYaw[vehicleid] = current_yaw;

    return 1;
}

// Returns in the unit degrees per second
// NOT the same units as SetVehicleAngularVelocity
// the units of this function are some dumb renderware mystery that I don't know
stock GetVehicleLocalAngularVelocity(vehicleid, &Float:vx, &Float:vy, &Float:vz) {
    vx = RollVelocity[vehicleid];
    vy = PitchVelocity[vehicleid];
    vz = YawVelocity[vehicleid];
}
