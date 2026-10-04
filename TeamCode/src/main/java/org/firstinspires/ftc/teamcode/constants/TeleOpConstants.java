package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.pedro.controllers.PIDFCoefficients;

@Configurable
public class TeleOpConstants {
    public static final double[] speeds = {0.2, 0.6, 1};
    public static final double baseSpeedMultiplier = 0.75;
    public static final double baseTurnSpeed = 2.5;
    public static final String BLUE_TELEOP_NAME = "🟦Blue🟦 Main";
    public static final String RED_TELEOP_NAME = "🟥Red🟥 Main";
    public static PIDFCoefficients teleopHeadingPID =
            new PIDFCoefficients(1, 0, .05, 0);
}
