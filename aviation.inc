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
