package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class RobotConstants {
    public static final ElapsedTime runtime = new ElapsedTime();
    static final double SMALL_WHEEL_DIAMETER = 3.77953;
    static final double WHEEL_DIAMETER_INCHES = SMALL_WHEEL_DIAMETER;
    static final double COUNTS_PER_MOTOR_REV = 537.6898395722;  // ((((1.0 + (46.0 / 17.0))) * (1.0 + (46.0 / 11.0))) * 28.0);
    static final double DRIVE_GEAR_REDUCTION = 1.0; // No External Gearing
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * PI);

    public static final double B = 1.1375;
    public static final double M = 0.889;

    public static final IMU.Parameters IMU_PARAMS = new IMU.Parameters(
        new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

    public static final double DRIVETRAIN_VELOCITY = 2000;

    public static final double[] speeds = {0.2, 0.6, 1};

    @Configurable
    public static class Reset {
        public static double SOFT_RESET_WAIT = .5;
        public static double HARD_RESET_WAIT = 1;
        public static double SNAP_THRESHOLD_DISTANCE = 24;
        public static double SNAP_THRESHOLD_HEADING = toRadians(5);
        public static double WALL_LOW = 11;
        public static double WALL_HIGH = 133;
    }

    public static final double baseSpeedMultiplier = 0.75;
    public static final double baseTurnSpeed = 2.5;

    @Configurable
    public static class Turret {
        public static com.pedropathing.control.PIDFCoefficients turretMotorPID =
            new com.pedropathing.control.PIDFCoefficients(.00055, 0, 0.00003, 0);
        public static com.pedropathing.control.PIDFCoefficients turretVelocityPID =
            new com.pedropathing.control.PIDFCoefficients(0.00003, 0, 0, 0);
        public static boolean USE_TURRET_VELOCITY_PID = false;
        public static double TURRET_ENCODERS_PER_DEGREE = 77.78;
        public static double TURRET_TOP_VEL = 10000; // encs per second
        public static double TURRET_FEEDFORWARD = 0 * TURRET_ENCODERS_PER_DEGREE / TURRET_TOP_VEL;
        public static double TURRET_STATIC_FEEDFORWARD = 0.055;
        public static double TURRET_FEEDFORWARD_SLOW_START = 1500;
        public static double TURRET_MAX_POWER = 0.75;
        public static double TURRET_OFFSET = 90; // degrees
        public static double TURRET_TS_OFFSET_ENC = 150; // length of touch sensor in encoder ticks
        public static double TURRET_MIN_POS = -2500;
        public static double TURRET_MAX_POS = 16500;
        public static double TURRET_SPEED_OFFSET = 3000;
        public static double TURRET_SPEED_MANUAL = 6000;
        public static boolean TURRET_ADJUST_FOR_VOLTAGE = false;
        public static int MAX_TIMES_NOT_RESET = 10;
    }

    public static com.pedropathing.control.PIDFCoefficients teleopHeadingPID =
        new com.pedropathing.control.PIDFCoefficients(1, 0, .05, 0);
    public static com.pedropathing.control.PIDFCoefficients launcherPIDF = new com.pedropathing.control.PIDFCoefficients(.002, 0, 0, .00055);
    public static PIDFCoefficients launcherReversePIDF = new PIDFCoefficients(80, 0, 0, 20);
    public static final double LAUNCHER_HEIGHT = 15.5;
    public static final double LAUNCHER_ANGLE = toRadians(50);
    public static double BALL_VEL_TO_MOTOR_VEL_COEFF = 4.45;
    public static double BALL_VEL_TO_MOTOR_VEL_CONST = 466;

    public static final String BLUE_TELEOP_NAME = "🟦Blue🟦 Main";
    public static final String RED_TELEOP_NAME = "🟥Red🟥 Main";

    public enum Dir {
        FORWARD, BACKWARD
    }

    public enum Color {
        RED, BLUE
    }

    public enum LEDColors {
        OFF, RED, ORANGE, YELLOW, SAGE, GREEN, AZURE, BLUE, INDIGO, VIOLET, WHITE;

        /**
         * Gets the position value for this LED color on the LED strip. <p>
         * Based on <a href="https://cdn11.bigcommerce.com/s-x56mtydx1w/images/stencil/original/products/2275/15126/3118-0808-0002-Product-Insight-4__88285.1757516465.png?c=1">this image</a>.
         *
         * @return Position value from 0.0 to 1.0 representing the color's position on the LED strip
         */
        public double position() {
            switch (this) {
                case RED:
                    return 0.277;
                case ORANGE:
                    return 0.333;
                case YELLOW:
                    return 0.388;
                case SAGE:
                    return 0.444;
                case GREEN:
                    return 0.500;
                case AZURE:
                    return 0.555;
                case BLUE:
                    return 0.611;
                case INDIGO:
                    return 0.666;
                case VIOLET:
                    return 0.722;
                case WHITE:
                    return 1.0;
                case OFF:
                default:
                    return 0.0;
            }
        }
    }
}
