package org.firstinspires.ftc.teamcode.constants;

import static java.lang.Math.toRadians;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.pedro.controllers.PIDFCoefficients;

@Configurable
public class LaunchConstants {
    public static final double LAUNCHER_HEIGHT = 15.5;
    public static final double LAUNCHER_ANGLE = toRadians(50);
    public static PIDFCoefficients launcherPIDF = new PIDFCoefficients(.002, 0, 0, .00055);
    public static com.qualcomm.robotcore.hardware.PIDFCoefficients launcherReversePIDF =
            new com.qualcomm.robotcore.hardware.PIDFCoefficients(80, 0, 0, 20);
    public static double BALL_VEL_TO_MOTOR_VEL_COEFF = 4.45;
    public static double BALL_VEL_TO_MOTOR_VEL_CONST = 466;
}
