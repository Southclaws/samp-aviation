// Returns in feet
stock Float:GetAltitudeForILS(
    Float:ils_x,
    Float:ils_y,
    Float:ils_z,

    Float:distance, // metres

    Float:glide_slope = 3.0 // standard glide slope of 3⁰
) {
    return 3.28084 * distance * floatsin(glide_slope, degrees);
}
