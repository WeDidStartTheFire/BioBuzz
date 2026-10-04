package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.MatchContext.Mode.AUTO;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.PrintLevel.DEBUG;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.PrintLevel.INFO;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.PrintLevel.VERBOSE;
import static org.firstinspires.ftc.teamcode.Utils.saveOdometryPosition;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MatchContext;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Turret;

public abstract class BaseAuto2 extends OpMode {
    protected Robot robot;
    protected Follower follower;
    protected IntakeController intakeController;
    protected Pose startPose, shootPose;
    protected TelemetryUtils tm;
    protected Color color;
    protected String name;
    private double lastUpdateTime = 0;
    private int totalMs = 0;
    private int totalUpdates = 0;

    protected abstract Command getAutoRoutine();

    protected abstract void configure();

    protected void configure(Pose startPose, Color color, String name) {
        this.startPose = startPose;
        this.color = color;
        this.name = name;
    }

    protected void onInit() {
    }

    protected void onStart() {
    }

    protected void onLoop() {
    }

    protected void onStop() {
    }

    @Override
    public final void init() {
        configure();
        robot = new Robot(hardwareMap, telemetry, new MatchContext(AUTO, color));
        follower = robot.drivetrain.follower;
        follower.setPose(startPose);
        tm = robot.drivetrain.tm;
        intakeController = new IntakeController(robot.intake, tm);
        tm.print(name + " auto initialized", INFO);
        tm.update();
        robot.initBulkCache();
        Scheduler.reset();
        onInit();
    }

    @Override
    public final void start() {
        follower.setPose(startPose);
        robot.limelight.start();
        robot.turret.setTarget(Turret.Target.GOAL);
        Scheduler.schedule(getAutoRoutine());
        onStart();
    }

    @Override
    public final void loop() {
        robot.updateBulkCache();
        robot.drivetrain.update();
        onLoop();
        Scheduler.execute();
        robot.turret.update(true);
        intakeController.update();
        robot.led.update();

        tm.drawRobot(follower);
        tm.print("Intake State", intakeController.getState(), INFO);
        Pose pose = robot.drivetrain.pose();
        if (pose != null) tm.print(pose, DEBUG);
        tm.print("Motor Goal Vel", robot.launcher.getGoalVel(shootPose, null), VERBOSE);
        tm.print("Launcher Vel", robot.launcher.getCachedVel(), VERBOSE);
        double t = getRuntime();
        if (lastUpdateTime != 0) {
            int ms = (int) ((t - lastUpdateTime) * 1000);
            totalMs += ms;
            totalUpdates++;
            tm.print("dt (ms)", ms, VERBOSE);
            tm.print("avg dt (ms)", totalMs / totalUpdates, VERBOSE);
        }
        lastUpdateTime = t;
        tm.updateOnlyPanels(5);
    }

    @Override
    public final void stop() {
        robot.drivetrain.update();
        follower.stop();
        Pose pose = robot.drivetrain.pose();
        if (pose != null) saveOdometryPosition(pose);
        intakeController.stop();
        robot.limelight.stop();
        tm.showLogs();
        tm.forceUpdate();
        onStop();
    }
}
