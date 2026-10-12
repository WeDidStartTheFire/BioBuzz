package org.firstinspires.ftc.teamcode.teleops.other;

import static org.firstinspires.ftc.teamcode.MatchContext.Mode.TELEOP;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.PrintLevel.INFO;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.PrintLevel.VERBOSE;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;
import static java.lang.Thread.sleep;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MatchContext;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.enums.RobotEnvironment;
import org.firstinspires.ftc.teamcode.pedro.controllers.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Turret;


@SuppressWarnings("CanBeFinal")
@TeleOp(name = "Tune Turret PIDF", group = "D")
@Configurable
@Disabled
public class TeleOp_TuneTurretPIDF extends OpMode {

    @IgnoreConfigurable
    Robot robot;
    @IgnoreConfigurable
    TelemetryUtils tm;

    // TODO: Set encoder goals based on numbers that make sense from telemetry
    public static int encoderGoalA = 0;
    public static int enocderGoalB = 5000;
    public static int encoderGoal = encoderGoalA;
    public static int increase = 1000;
    public static int delay = 50;

    public static double P = 0.00055;
    public static double D = 0.00003;
    public static double F = 0;
    public static double maxPower = 0.75;
    @IgnoreConfigurable
    PIDFCoefficients pidf = new PIDFCoefficients(P, 0, D, 0);
    @IgnoreConfigurable
    double[] increments = {10, 1, .1, .01, .001};
    public static int incIdx = 1;
    private double t;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, new MatchContext(TELEOP, Color.BLUE));
        robot.drivetrain.setPose(loadOdometryPosition());
        robot.initBulkCache();
        tm = robot.drivetrain.tm;
        tm.setEnvironment(RobotEnvironment.DEBUG);
        tm.print("Tune Turret PIDF Initialized", INFO);
        tm.update();
        t = getRuntime();
    }

    @Override
    public void loop() {
        robot.turret.setTarget(Turret.Target.HOLD);
        robot.updateBulkCache();
        double dt = getRuntime() - t;
        t = getRuntime();
        if (gamepad1.bWasPressed()) incIdx = (incIdx + 1) % increments.length;
        if (gamepad1.aWasPressed())
            encoderGoal = encoderGoal == encoderGoalA ? enocderGoalB : encoderGoalA;
        if (gamepad1.dpadUpWasPressed()) D += increments[incIdx];
        if (gamepad1.dpadDownWasPressed()) D -= increments[incIdx];
        if (gamepad1.dpadRightWasPressed()) P += increments[incIdx];
        if (gamepad1.dpadLeftWasPressed()) P -= increments[incIdx];
        if (gamepad1.x) encoderGoal += (int) (increase * dt);
        if (gamepad1.y) encoderGoal -= (int) (increase * dt);

        TurretConstants.TURRET_MAX_POWER = maxPower;
        pidf = new PIDFCoefficients(P, 0, D, F);
        robot.turret.turretPIDController.setCoefficients(pidf);
        robot.turret.turretPIDController.setTarget(encoderGoal);
        robot.turret.update(gamepad2.left_stick_button);
        double pos = robot.turret.getEncoderPosition();
        double error = pos - encoderGoal;
        tm.print("Position", pos, VERBOSE);
        tm.print("Target", encoderGoal, VERBOSE);
        tm.print("Error", error, VERBOSE);
        tm.print("---------------------------", VERBOSE);
        tm.print("P", P, INFO);
        tm.print("D", D, INFO);
        tm.print("Increment", increments[incIdx], INFO);
        tm.print("---------------------------", INFO);
        tm.print("PIDF", pidf, VERBOSE);
        DcMotor.RunMode turretMode = robot.turret.getRunMode();
        if (turretMode != null) tm.print("Run Mode", turretMode, INFO);
        tm.print("dt (ms)", dt * 1000, INFO);
        tm.updateOnlyPanels();
        try {
            sleep(delay);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
