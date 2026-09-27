package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class TurretConstants {
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
