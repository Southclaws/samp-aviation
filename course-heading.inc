// Returns the recommended heading to fly in order to capture a described course
// situated on the specified radial around the specified waypoint.
//
// Can be used to calculate VOR navigation or ILS approaches.
//
// For a VOR, the waypoint coordinates represent the VOR beacon and the radial
// is the TO radial for the VOR. This function does not currently support FROM
// based courses (back-course).
//
// For an ILS, the waypoint is the ILS beacon on the end of the runway and the
// radial is the runway heading.
//
// procedure_radius is a somewhat arbitrary term to describe a distance used for
// generating bezier points for plotting a course. 500m is a generally good
// value but it may be larger if you want to give certain aircraft more time to
// make their turns. It essentially means that the bezier curve shouldn't be
// generated to be too sharp of a turn.
stock Float:GetHeadingForWaypointCourse(
    Float:aircraft_x,
    Float:aircraft_y,
    Float:waypoint_x,
    Float:waypoint_y,
    Float:radial,
    Float:procedure_radius
) {
    new Float:distance_to_waypoint = GetDistance2D(
        aircraft_x, aircraft_y,
        waypoint_x, waypoint_y
    );

    new Float:glide_slope_curve_distance = distance_to_waypoint -
        (procedure_radius * 0.5);
    new Float:glide_slope_entry_distance = distance_to_waypoint -
        (procedure_radius * 2);

    new
        Float:course_entry_x = waypoint_x + (
            glide_slope_curve_distance *
            floatsin(-(radial - 180.0), degrees) * floatcos(3.0, degrees)
        ),
        Float:course_entry_y = waypoint_y + (
            glide_slope_curve_distance *
            floatcos(-(radial - 180.0), degrees) * floatcos(3.0, degrees)
        ),
        Float:course_glide_x = waypoint_x + (
            glide_slope_entry_distance *
            floatsin(-(radial - 180.0), degrees) * floatcos(3.0, degrees)
        ),
        Float:course_glide_y = waypoint_y + (
            glide_slope_entry_distance *
            floatcos(-(radial - 180.0), degrees) * floatcos(3.0, degrees)
        ),
        Float:target_x,
        Float:target_y,
        Float:course_target_heading;

    BezierOrder2(
        aircraft_x,
        aircraft_y,

        course_entry_x,
        course_entry_y,

        course_glide_x,
        course_glide_y,

        0.01,

        target_x,
        target_y,
        course_target_heading
    );

    return course_target_heading;
}
