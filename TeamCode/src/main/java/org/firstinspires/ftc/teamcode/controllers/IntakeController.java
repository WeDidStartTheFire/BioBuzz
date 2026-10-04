package org.firstinspires.ftc.teamcode.controllers;

import com.pedropathing.utils.Timer;

import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Intake;

import java.util.concurrent.TimeUnit;

public class IntakeController {

    private State state;
    private final Timer stateTimer = new Timer();

    private boolean isBusy;
    private final Intake intake;
    private final TelemetryUtils tm;

    private enum State {
        IDLE,
        INTAKE,
        MANUAL_INTAKE,
        OUTTAKE,
    }

    public IntakeController(Intake intake, TelemetryUtils tm) {
        this.tm = tm;
        setState(State.IDLE);
        this.intake = intake;
        isBusy = false;
    }

    /**
     * Updates the LEDs and the IntakeController state machine:<p>
     * INTAKE -> IDLE: When indexer is full<p>
     * Other transitions controlled via calling methods
     *
     * @see #intake()
     * @see #outtake()
     * @see #stop()
     */
    public void update() {
        switch (state) {
            case IDLE:
                isBusy = false;
                intake.power(0);
                break;
            case MANUAL_INTAKE:
            case INTAKE:
                intake.power(-1);
                break;
            case OUTTAKE:
                intake.power(1);
                break;
        }
    }

    /**
     * @return Whether the robot is actively intaking
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

    public void manualIntake() {
        isBusy = true;
        setState(State.MANUAL_INTAKE);
    }

    /**
     * Stops the intake
     */
    public void stop() {
        if (state != State.IDLE) setState(State.IDLE);
        isBusy = false;
    }

    private void setState(State state) {
        if (state != null && state != this.state)
            tm.log("IntakeController: " + this.state + " -> " + state, stateTimer.get(TimeUnit.SECONDS));
        setStateNoWait(state);
        this.stateTimer.reset();
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
