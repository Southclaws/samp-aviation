// Returns target altitude for an ILS approach glide slope based on just the
// distance from the ILS beacon. This value should be added to the beacon's
// altitude and then converted to feet for an actual target altitude.
stock Float:GetAltitudeForILS(
    Float:distance, // metres
    Float:glide_slope = 3.0 // standard glide slope of 3⁰
) {
    return distance * floatsin(glide_slope, degrees);
}
