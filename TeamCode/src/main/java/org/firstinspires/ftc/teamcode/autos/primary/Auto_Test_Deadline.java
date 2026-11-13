package org.firstinspires.ftc.teamcode.autos.primary;

import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;
import com.pedropathing.utils.Timer;

import org.firstinspires.ftc.teamcode.autos.BaseAuto2;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Intake.Behavior;

public class Auto_Test_Deadline extends BaseAuto2<org.firstinspires.ftc.teamcode.autos.paths.Paths_Red> {

    public static final String baseName = "Deadline";
    public static final String group = "A";
    public Timer timer = new Timer();

    public enum States {
        SHOOT,
        INTAKE,
        IDLE
    }

    public static States currState;

    @Override
    protected void configure() {
        configure(org.firstinspires.ftc.teamcode.autos.paths.Paths_Red.class, Color.RED, "Test Deadline");
    }

    private static Command setState(States state) {
        return Command.build()
                .setStart(() -> {
                    currState = state;
                }).setDone(() -> currState == state);
    }

    private static Command wait(int time, Timer timer) {
        return Command.build()
                .setStart(timer::reset)
                .setDone(() -> {
                    return timer.seconds() >= time;
                });
    }

    private static Command stateController(Robot robot) {
        return Command.build()
                .setExecute(() -> {
                    switch (currState) {
                        case SHOOT:
                            robot.launcher.spin();
                            break;
                        case INTAKE:
                            robot.intake.setBehavior(Behavior.INTAKE);
                            break;
                        case IDLE:
                            if (robot.launcher.isSpinning(robot.launcher.getVel())) {
                                robot.launcher.stop();
                            } else if (robot.intake.getBehavior() == Behavior.INTAKE) {
                                robot.intake.setBehavior(Behavior.OFF);
                            }
                            break;
                    }
                }).setDone(() -> false);
    }

    private Command followPaths() {
        return sequential(
                follow(follower, paths.startToShoot1()),
                setState(States.SHOOT),
                wait(3, timer),
                setState(States.IDLE),
                follow(follower, paths.shootToPrepPickup()),
                setState(States.INTAKE),
                wait(1, timer),
                follow(follower, paths.prepPickupToPickup()),
                setState(States.IDLE),
                follow(follower, paths.pickupToBackUp()),
                follow(follower, paths.backUpToShoot2()),
                setState(States.SHOOT),
                wait(3, timer),
                setState(States.IDLE),
                follow(follower, paths.shoot2ToPark1()),
                follow(follower, paths.park1ToPark2()),
                follow(follower, paths.park2ToPark3())
        );
    }

    @Override
    protected Command getAutoRoutine() {
        return deadline(
                deadline(
                        followPaths(), // Deadline, everything will stop when this is finished.
                        stateController(robot)
                )
        );
    }
}


