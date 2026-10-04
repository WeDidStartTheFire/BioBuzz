package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.MatchContext.Mode.AUTO;
import static org.firstinspires.ftc.teamcode.Utils.saveOdometryPosition;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MatchContext;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Turret;

import java.util.concurrent.TimeUnit;

public abstract class BaseAuto<S extends Enum<S>> extends OpMode {
    protected Robot robot;
    protected IntakeController intakeController;
    protected Pose startPose, shootPose;
    protected TelemetryUtils tm;
    protected Timer stateTimer = new Timer();
    protected S state;
    protected S initialState;
    protected Color color;
    protected String name;
    private double lastUpdateTime = 0;
    private int totalMs = 0;
    private int totalUpdates = 0;

    protected abstract void buildPaths();

    protected abstract void pathUpdate();

    protected abstract void configure();

    protected void onInit() {
    }

    protected void onStart() {
    }

    protected void onStop() {
    }

    protected void setState(S state) {
        tm.log(this.state + " -> " + state, stateTimer.get(TimeUnit.SECONDS));
        setStateNoWait(state);
        stateTimer.reset();
    }

    protected void setStateNoWait(S state) {
        this.state = state;
    }

    @Override
    public final void init() {
        configure();
        robot = new Robot(hardwareMap, telemetry, new MatchContext(AUTO, color));
        robot.drivetrain.follower.setPose(startPose);
        tm = robot.drivetrain.tm;
        buildPaths();
        intakeController = new IntakeController(robot);
        tm.print(name + " auto initialized");
        tm.update();
        setStateNoWait(initialState);
        robot.initBulkCache();
        Scheduler.reset();
        onInit();
    }

    @Override
    public final void start() {
        robot.drivetrain.follower.setPose(startPose);
        robot.limelight.start();
        robot.turret.setTarget(Turret.Target.GOAL);
        setState(initialState);
        onStart();
    }

    @Override
    public final void loop() {
        robot.updateBulkCache();
        robot.drivetrain.follower.update();
        RobotState.pose = robot.drivetrain.follower.pose();
        pathUpdate();
        robot.turret.update(true);
        intakeController.update();
        robot.led.update();

        tm.drawRobot(robot.drivetrain.follower, 250);
        tm.print("Path State", state);
        tm.print("Intake State", intakeController.getState());
        if (robot.drivetrain.follower.pose() != null) tm.print(robot.drivetrain.follower.pose());
        tm.print("Motor Goal Vel", robot.launcher.getGoalVel(shootPose, null));
        tm.print("Launcher Vel", robot.launcher.getCachedVel());
        double t = getRuntime();
        if (lastUpdateTime != 0) {
            int ms = (int) ((t - lastUpdateTime) * 1000);
            totalMs += ms;
            totalUpdates++;
            tm.print("dt (ms)", ms);
            tm.print("avg dt (ms)", totalMs / totalUpdates);
        }
        lastUpdateTime = t;
        tm.updateOnlyPanels(5);
    }

    @Override
    public final void stop() {
        robot.drivetrain.follower.update();
        robot.drivetrain.follower.stop();
        if (robot.drivetrain.follower.pose() != null)
            saveOdometryPosition(robot.drivetrain.follower.pose());
        intakeController.stop();
        robot.limelight.stop();
        tm.showLogs();
        tm.update();
        onStop();
    }
}
