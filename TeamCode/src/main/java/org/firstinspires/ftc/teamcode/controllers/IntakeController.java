package org.firstinspires.ftc.teamcode.controllers;

import static org.firstinspires.ftc.teamcode.RobotConstants.LEDColors.AZURE;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEDColors.BLUE;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEDColors.GREEN;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.robot.mechanisms.LED;

public class IntakeController {

    private State state;
    private final Timer stateTimer = new Timer();

    private boolean isBusy;
    private final Robot robot;
    private final Timer artifactDetectedTimer = new Timer();
    private final TelemetryUtils tm;

    private enum State {
        IDLE,
        INTAKE,
        OUTTAKE,
    }

    public IntakeController(Robot robot) {
        tm = robot.drivetrain.tm;
        setState(State.IDLE);
        this.robot = robot;
        isBusy = false;
    }

    /**
     * Updates the LEDs and the IntakeController state machine:<p>
     * INTAKE -> IDLE: When indexer is full<p>
     * Other transitions controlled via calling methods
     * @see #intake()
     * @see #outtake()
     * @see #stop()
     */
    public void update() {
        if (robot.indexer.getTotalArtifacts() == 3)
            robot.led.setColor(GREEN, isBusy ? LED.Priority.HIGH : LED.Priority.MEDIUM);
        else if (robot.indexer.getTotalArtifacts() == 2)
            robot.led.setColor(AZURE, isBusy ? LED.Priority.MEDIUM : LED.Priority.LOW);
        else if (robot.indexer.getTotalArtifacts() <= 1)
            robot.led.setColor(BLUE, LED.Priority.LOW);
        switch (state) {
            case IDLE:
                RobotState.normalIntaking = false;
                isBusy = false;
                robot.intake.setBehavior(Intake.Behavior.OFF);
                break;
            case INTAKE:
                RobotState.normalIntaking = true;
                robot.intake.setBehavior(Intake.Behavior.INTAKE);
                break;
            case OUTTAKE:
                RobotState.normalIntaking = false;
                robot.intake.setBehavior(Intake.Behavior.OUTTAKE);
                break;
        }
    }

    /**
     * @return Whether the robot is actively intaking artifacts
     */
    public boolean isBusy() {
        return isBusy;
    }

    /**
     * Turns outtaking on
     */
    public void outtake() {
        isBusy = true;
        setState(State.OUTTAKE);
    }

    /**
     * Turns intaking on
     */
    public void intake() {
        isBusy = true;
        setState(State.INTAKE);
    }

    /**
     * Stops the intake
     */
    public void stop() {
        if (state != State.IDLE) setState(State.IDLE);
        isBusy = false;
    }

    /**
     * Manually stops the motor instead of just changing the state.
     */
    public void forceStop() {
        robot.intake.stop();
        setState(State.IDLE);
    }

    private void setState(State state) {
        if (state != null && state != this.state)
            tm.log("IntakeController: " + this.state + " -> " + state, stateTimer.getElapsedTimeSeconds());
        setStateNoWait(state);
        this.stateTimer.resetTimer();
    }

    private void setStateNoWait(State state) {
        this.state = state;
    }

    /**
     * Gets the IntakeController's current state
     *
     * @return Current IntakeController state as a String
     */
    public String getState() {
        return state.toString();
    }
}
