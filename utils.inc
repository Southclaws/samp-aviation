// simple linear interpolation
Float:LinierInterpolation(Float:v0, Float:v1, Float:t) {
    return (1 - t) * v0 + t * v1;
}

// An order 2 bezier curve function.
stock Float:BezierOrder2(
    Float:p1_x,
    Float:p1_y,

    Float:p2_x,
    Float:p2_y,

    Float:p3_x,
    Float:p3_y,

    Float:t,

    &Float:t_x,
    &Float:t_y,
    &Float:t_h
) {
    new
        Float:l1_x = LinierInterpolation(p1_x, p2_x, t),
        Float:l1_y = LinierInterpolation(p1_y, p2_y, t),
        Float:l2_x = LinierInterpolation(p2_x, p3_x, t),
        Float:l2_y = LinierInterpolation(p2_y, p3_y, t);
    
    t_x = LinierInterpolation(l1_x, l2_x, t);
    t_y = LinierInterpolation(l1_y, l2_y, t);
    t_h = GetAngleToPoint(l1_x, l1_y, l2_x, l2_y);
}
