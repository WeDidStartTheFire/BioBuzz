package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Intake;

public class AutoStates {

    private final Robot robot;

    public enum State {
        SHOOT,
        INTAKE,
        IDLE
    }

    public AutoStates(Robot robot) {
        this.robot = robot;
    }

    public static State currentState = State.IDLE;

    public Command setCurrentState(State state) {
        return Command.build()
                .setStart(() -> currentState = state).setDone(() -> currentState == state);
    }

    public void update() {
        switch (currentState) {
            case SHOOT:
                robot.launcher.spin();
                break;
            case INTAKE:
                robot.intake.behavior = Intake.Behavior.INTAKE;
                break;
            case IDLE:
                if (robot.launcher.isSpinning(robot.launcher.getVel())) {
                    robot.launcher.stop();
                }
                if (robot.intake.behavior == Intake.Behavior.INTAKE) {
                    robot.intake.behavior = Intake.Behavior.OFF;
                }
                break;
        }
    }

}
