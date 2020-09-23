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
