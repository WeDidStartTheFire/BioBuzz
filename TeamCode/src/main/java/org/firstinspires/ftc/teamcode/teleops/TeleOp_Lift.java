package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "TeleOp_Lift", group = "A")
public class TeleOp_Lift extends OpMode {

    private TelemetryUtils tm;
    private Robot robot;
    private Lift lift;

    enum State {
        UP,
        DOWN,
        IDLE
    }

    State currentState;

    private void updateStates() {
        switch (currentState) {
            case UP:
                lift.setLiftMotorPower(1.0f);
                break;
            case DOWN:
                lift.setLiftMotorPower(-1.0f);
                break;
            case IDLE:
                lift.setLiftMotorPower(0.0f);
                break;
        }
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, false);
        tm = new TelemetryUtils(telemetry);
        lift = new Lift(hardwareMap, tm);
        currentState = State.IDLE;
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) { currentState = State.UP; }
        else if (gamepad1.dpadUpWasReleased()) { currentState = State.IDLE; }
        else if (gamepad1.dpadDownWasPressed()) { currentState = State.DOWN; }
        else if (gamepad1.dpadDownWasReleased()) { currentState = State.IDLE; }
        updateStates();
    }
}
