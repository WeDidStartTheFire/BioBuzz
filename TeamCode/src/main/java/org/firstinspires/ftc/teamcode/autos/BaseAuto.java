package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.RobotConstants.runtime;
import static org.firstinspires.ftc.teamcode.RobotState.motif;
import static org.firstinspires.ftc.teamcode.RobotState.pose;
import static org.firstinspires.ftc.teamcode.RobotState.vel;
import static org.firstinspires.ftc.teamcode.Utils.saveOdometryPosition;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.controllers.LaunchController;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Turret;

public abstract class BaseAuto<S extends Enum<S>> extends OpMode {
    protected Robot robot;
    protected LaunchController launchController;
    protected IntakeController intakeController;
    protected Pose startPose, shootPose;
    protected TelemetryUtils tm;
    protected Timer stateTimer = new Timer();
    protected S state;
    protected S initialState;
    protected RobotConstants.Color color;
    protected String name;
    private long lastUpdateTime = 0;
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
        robot.stop();
    }

    protected void onUpdate() {
        robot.updateBulkCache();
        robot.drivetrain.follower.update();
        pose = robot.drivetrain.follower.getPose();
        vel = robot.drivetrain.follower.getVelocity();
        pathUpdate();
        Scheduler.execute();
        robot.update();
    }

    protected void setState(S state) {
        stateTimer.resetTimer();
        tm.log(this.state + " -> " + state, stateTimer.getElapsedTimeSeconds());
        setStateNoWait(state);
    }

    protected void setStateNoWait(S state) {
        this.state = state;
    }

    @Override
    public final void init() {
        configure();
        RobotState.auto = true;
        RobotState.color = color;
        robot = new Robot(hardwareMap, telemetry, true);
        robot.drivetrain.follower.setStartingPose(startPose);
        robot.indexer.markAllUnknown();
        tm = robot.drivetrain.tm;
        buildPaths();
        intakeController = new IntakeController(robot.intake, tm);
        tm.print(name + " auto initialized", INFO);
        tm.update();
        setStateNoWait(initialState);
        robot.initBulkCache();
        onInit();
    }

    @Override
    public final void start() {
        robot.drivetrain.follower.setPose(startPose);
        robot.feeder.retract();
        robot.indexer.setPos(0);
        robot.limelight.start();
        robot.turret.setTarget(Turret.Target.GOAL);
        RobotState.motif = robot.limelight.getMotif();
        setState(initialState);
        onStart();
    }

    @Override
    public final void loop() {
        onUpdate();

        tm.drawRobot(robot.drivetrain.follower);
        tm.print("Path State", state, INFO);
        tm.print("Intake State", intakeController.getState(), INFO);
        Pose pose = robot.drivetrain.pose();
        if (pose != null) tm.print(pose, DEBUG);
        tm.print("Motor Goal Vel", robot.launcher.getGoalVel(shootPose, null), VERBOSE);
        tm.print("Launcher Vel", robot.launcher.getCachedVel(), VERBOSE);
        double t = getRuntime();
        if (lastUpdateTime != 0) {
            int ms = Math.toIntExact((t - lastUpdateTime) / 1_000_000);
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
        robot.drivetrain.follower.breakFollowing();
        pose = robot.drivetrain.follower.getPose();
        if (pose != null) saveOdometryPosition(pose);
        tm.showLogs();
        tm.update();
        onStop();
    }
}
