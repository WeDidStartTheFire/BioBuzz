package org.firstinspires.ftc.teamcode.constants;

import static java.lang.Math.toRadians;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class LocalizationConstants {
    public static double MAX_LIN_VEL = 40;
    public static double MAX_ANG_VEL = toRadians(360);
    public static double MAX_AMBIGUITY = 1;
    public static double LERP_START = 6;
    public static double LERP_END_WEIGHT = 0.5;
    public static double MAX_DIST = 12;
    public static double MAX_TAG_DISTANCE = 72;
}
