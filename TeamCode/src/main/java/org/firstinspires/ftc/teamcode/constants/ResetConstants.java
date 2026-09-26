package org.firstinspires.ftc.teamcode.constants;

import static java.lang.Math.toRadians;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ResetConstants {
    public static double SOFT_RESET_WAIT = .5;
    public static double HARD_RESET_WAIT = 1;
    public static double SNAP_THRESHOLD_DISTANCE = 24;
    public static double SNAP_THRESHOLD_HEADING = toRadians(5);
    public static double WALL_LOW = 11;
    public static double WALL_HIGH = 133;
}
