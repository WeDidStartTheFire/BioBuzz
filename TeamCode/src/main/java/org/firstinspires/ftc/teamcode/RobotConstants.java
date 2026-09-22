package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TelemetryUtils.ErrorLevel.CRITICAL;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.ErrorLevel.HIGH;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.ErrorLevel.LOW;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.ErrorLevel.MEDIUM;
import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

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

    public static final double MIDDLE_INDEXER_POS = 0.455;
    public static double INDEXER_SPEED = 0.9;
    public static final double INDEXER_POS_EPSILON = 1e-4;

    public static final double[] speeds = {0.2, 0.6, 1};

    public enum Hardware {
        DRIVETRAIN_LEFT_FRONT_MOTOR("Left Front Drivetrain Motor", DcMotorEx.class, "leftFront", Hub.CONTROL, PortType.MOTOR, 1, CRITICAL),
        DRIVETRAIN_LEFT_BACK_MOTOR("Left Back Drivetrain Motor", DcMotorEx.class, "leftBack", Hub.CONTROL, PortType.MOTOR, 3, CRITICAL),
        DRIVETRAIN_RIGHT_FRONT_MOTOR("Right Front Drivetrain Motor", DcMotorEx.class, "rightFront", Hub.CONTROL, PortType.MOTOR, 0, CRITICAL),
        DRIVETRAIN_RIGHT_BACK_MOTOR("Right Back Drivetrain Motor", DcMotorEx.class, "rightBack", Hub.CONTROL, PortType.MOTOR, 2, CRITICAL),
        FEEDER_SERVO_A("Feeder Servo A", Servo.class, "feederServoA", Hub.EXPANSION, PortType.SERVO, 0, HIGH),
        FEEDER_SERVO_B("Feeder Servo B", Servo.class, "feederServoB", Hub.EXPANSION, PortType.SERVO, 1, HIGH),
        COLOR_SENSOR_A("Color Sensor A", RevColorSensorV3.class, "colorSensorA", Hub.CONTROL, PortType.I2C, 2, CRITICAL),
        COLOR_SENSOR_B("Color Sensor B", RevColorSensorV3.class, "colorSensorB", Hub.CONTROL, PortType.I2C, 3, CRITICAL),
        LED("LED", Servo.class, "led", Hub.CONTROL, PortType.SERVO, 0, CRITICAL),
        INDEXER_SERVO("Indexer Servo", Servo.class, "indexerServo", Hub.EXPANSION, PortType.SERVO, 2, CRITICAL),
        TURRET_MOTOR("Turret Motor", DcMotorEx.class, "turretMotor", Hub.EXPANSION, PortType.MOTOR, 0, HIGH),
        TOUCH_SENSOR_A("Touch Sensor A", TouchSensor.class, "touchSensorA", Hub.UNKNOWN, PortType.DIGITAL, -1, LOW),
        TOUCH_SENSOR_B("Touch Sensor B", TouchSensor.class, "touchSensorB", Hub.UNKNOWN, PortType.DIGITAL, -1, LOW),
        TURRET_TOUCH_SENSOR("Turret Touch Sensor", TouchSensor.class, "turretTouchSensor", Hub.EXPANSION, PortType.DIGITAL, 2, HIGH),
        LAUNCHER_MOTOR_A("Launcher Motor A", DcMotorEx.class, "launcherMotorA", Hub.EXPANSION, PortType.MOTOR, 1, CRITICAL),
        LAUNCHER_MOTOR_B("Launcher Motor B", DcMotorEx.class, "launcherMotorB", Hub.EXPANSION, PortType.MOTOR, 2, CRITICAL),
        LIMELIGHT("Limelight", Limelight3A.class, "limelight", Hub.CONTROL, PortType.USB, -1, MEDIUM),
        SPARKFUN_OTOS("SparkFun OTOS", SparkFunOTOS.class, "sensorOtos", Hub.CONTROL, PortType.I2C, 1, CRITICAL),
        INTAKE_MOTOR("Intake Motor", DcMotorEx.class, "intakeMotor", Hub.EXPANSION, PortType.MOTOR, 0, HIGH),
        INTAKE_SERVO_A("Intake Servo A", CRServo.class, "intakeServoA", Hub.EXPANSION, PortType.SERVO, 3, HIGH),
        INTAKE_SERVO_B("Intake Servo B (roller)", CRServo.class, "intakeServoB", Hub.EXPANSION, PortType.SERVO, 4, LOW),
        INTAKE_SERVO_C("Intake Servo C", CRServo.class, "intakeServoC", Hub.EXPANSION, PortType.SERVO, 5, HIGH),
        IMU("IMU", IMU.class, "imu", Hub.CONTROL, PortType.I2C, 0, MEDIUM);

        public enum Hub {
            CONTROL, EXPANSION, UNKNOWN;

            @NonNull
            public String toString() {
                switch (this) {
                    case CONTROL:
                        return "Control Hub";
                    case EXPANSION:
                        return "Expansion Hub";
                    default:
                        return "Unknown Hub";
                }
            }
        }

        public enum PortType {
            SERVO, MOTOR, DIGITAL, I2C, USB;

            @NonNull
            public String toString() {
                switch (this) {
                    case SERVO:
                        return "Servo Port";
                    case MOTOR:
                        return "Motor Port";
                    case DIGITAL:
                        return "Digital Port";
                    case I2C:
                        return "I2C Port";
                    case USB:
                        return "USB Port";
                    default:
                        return "Unknown Port";
                }
            }
        }

        public final String name;
        public final Class<?> classOrInterface;
        public final String deviceName;
        public final Hub hub;
        public final PortType portType;
        public final int port;
        public final TelemetryUtils.ErrorLevel errorLevel;

        Hardware(String name, Class<?> classOrInterface, String deviceName, Hub hub, PortType portType, int port, TelemetryUtils.ErrorLevel errorLevel) {
            this.name = name;
            this.classOrInterface = classOrInterface;
            this.deviceName = deviceName;
            this.hub = hub;
            this.portType = portType;
            this.port = port;
            this.errorLevel = errorLevel;
        }

        public String getWarnMessage() {
            return name + " is disconnected. Please check " + hub.toString() + " " + portType.toString() + " " + port;
        }
    }

    @Configurable
    public static class LaunchController {
        public static double MAX_LAUNCHER_SPIN_WAIT = 5;
        public static double MAX_DROOP_WAIT = 3;
        public static double MAX_FEEDER_DOWN_WAIT = .35;
        public static double MIN_FEEDER_DOWN_WAIT = .15;
        public static double ARTIFACT_LAUNCH_WAIT = .15;
        public static int MAX_FAILED_ATTEMPTS = 5;
        public static double STOP_LAUNCHER_WAIT = 0.25;
        public static int MAX_ARTIFACT_PRESENT_COUNT = 1;
    }

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
    public static class Autonomous {
        public static PathConstraints slowIntakePathConstraints = new PathConstraints(
            0.3,
            500,
            0.5,
            0.4
        );
        public static double INTAKE_MOVE_MAX_SPEED = 0.4;
        public static double MAX_MOTIF_DETECT_WAIT = 1;
        public static double MAX_INTAKE_PATH_WAIT = 2.5;
        public static double INTAKE_AFTER_LAUNCH_WAIT = 1;
    }

    public static double INDEXER_ARTIFACT_DETECTION_WAIT = 0.9;
    public static double PARTIAL_INDEXER_ARTIFACT_DETECTION_WAIT = 0.2;

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

    @Configurable
    public static class Positions {
        public static Pose3D RED_GOAL_POSE = new Pose3D(new Position(DistanceUnit.INCH, 139, 139, 44, 0),
            new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, 0));
        public static Pose3D BLUE_GOAL_POSE = new Pose3D(new Position(DistanceUnit.INCH, 5, 139, 44, 0),
            new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, 0));
        public static Pose RED_HUMAN_PLAYER_POSE = new Pose(0, 9);
        public static Pose BLUE_HUMAN_PLAYER_POSE = new Pose(144, 9);

        public static Pose RED_BASE_ZONE = new Pose(38, 33, toRadians(90));
        public static Pose BLUE_BASE_ZONE = new Pose(106, 33, toRadians(90));
        public static Pose RED_FAR_LAUNCH = new Pose(84, 20, toRadians(90));
        public static Pose BLUE_FAR_LAUNCH = new Pose(60, 20, toRadians(90));
        public static Pose RED_HUMAN_PLAYER = new Pose(20, 14, toRadians(90));
        public static Pose BLUE_HUMAN_PLAYER = new Pose(124, 14, toRadians(90));
    }

    public static final String BLUE_TELEOP_NAME = "🟦Blue🟦 Main";
    public static final String RED_TELEOP_NAME = "🟥Red🟥 Main";

    public enum Dir {
        FORWARD, BACKWARD
    }

    public enum Color {
        RED, BLUE
    }

    public enum HivePosition {
        STAGE, TRANSITIONING, AUDIENCE, UNKNOWN
    }

    public enum Motif {
        GPP, PPG, PGP, UNKNOWN;

        @NonNull
        @Override
        public String toString() {
            switch (this) {
                case GPP:
                    return "🟢🟣🟣";
                case PGP:
                    return "🟣🟢🟣";
                case PPG:
                    return "🟣🟣🟢";
                case UNKNOWN:
                    return "Not Detected ⚫⚫⚫";
                default:
                    return super.toString();
            }
        }

        /**
         * Gets the nth artifact in this motif sequence (0-indexed).
         *
         * @param n The position in the motif sequence (0-based, wraps around)
         * @return The artifact at the specified position
         */
        public Artifact getNthArtifact(int n) {
            n %= 3;
            switch (this) {
                case GPP:
                    switch (n) {
                        case 0:
                            return Artifact.GREEN;
                        case 1:
                        case 2:
                            return Artifact.PURPLE;
                    }
                    break;
                case PGP:
                    switch (n) {
                        case 1:
                            return Artifact.GREEN;
                        case 0:
                        case 2:
                            return Artifact.PURPLE;
                    }
                    break;
                case PPG:
                    switch (n) {
                        case 2:
                            return Artifact.GREEN;
                        case 1:
                        case 0:
                            return Artifact.PURPLE;
                    }
                    break;
            }
            return Artifact.UNKNOWN;
        }
    }

    public enum Artifact {
        GREEN,
        PURPLE,
        EMPTY,
        UNKNOWN;

        @NonNull
        @Override
        public String toString() {
            switch (this) {
                case GREEN:
                    return "🟢Green🟢";
                case PURPLE:
                    return "🟣Purple🟣";
                case EMPTY:
                    return "◌Empty◌";
                case UNKNOWN:
                    return "?UNKNOWN?";
                default:
                    return super.toString();
            }
        }
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
