package org.firstinspires.ftc.teamcode.constants;

import static java.lang.Math.toRadians;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Configurable
public class LaunchConstants {
    public static final double LAUNCHER_HEIGHT = 15.5;
    public static final double LAUNCHER_ANGLE = toRadians(50);
    public static double MAX_LAUNCHER_SPIN_WAIT = 5;
    public static double MAX_DROOP_WAIT = 3;
    public static double MAX_FEEDER_DOWN_WAIT = .35;
    public static double MIN_FEEDER_DOWN_WAIT = .15;
    public static double ARTIFACT_LAUNCH_WAIT = .15;
    public static int MAX_FAILED_ATTEMPTS = 5;
    public static double STOP_LAUNCHER_WAIT = 0.25;
    public static int MAX_ARTIFACT_PRESENT_COUNT = 1;
    public static com.pedropathing.control.PIDFCoefficients launcherPIDF = new com.pedropathing.control.PIDFCoefficients(.002, 0, 0, .00055);
    public static PIDFCoefficients launcherReversePIDF = new PIDFCoefficients(80, 0, 0, 20);
    public static double BALL_VEL_TO_MOTOR_VEL_COEFF = 4.45;
    public static double BALL_VEL_TO_MOTOR_VEL_CONST = 466;
}
