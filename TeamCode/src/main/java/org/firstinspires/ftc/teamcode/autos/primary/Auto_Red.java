package org.firstinspires.ftc.teamcode.autos.primary;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;
import com.pedropathing.utils.Timer;

import org.firstinspires.ftc.teamcode.autos.BaseAuto2;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Intake;

public class Auto_Red extends BaseAuto2<org.firstinspires.ftc.teamcode.autos.paths.Paths_Red> {
    public static final String baseName = "Red";
    public static final String group = "A";
    protected final Class<org.firstinspires.ftc.teamcode.autos.paths.Paths_Red> autoPathsClass = org.firstinspires.ftc.teamcode.autos.paths.Paths_Red.class;

    // Written in the documentation say that we can create a Command class to store commands like
    // these in. It may be a good option as we will probably use commands like these multiple
    // times. It's only like this for testing reasons.
    private static Command shoot(Robot robot) {
        Timer timer = new Timer();

        return Command.build()
                .setStart(timer::reset)
                .setExecute(robot.launcher::spin)
                .setDone(() -> timer.seconds() == 3)
                .setEnd(endCondition -> {
                    robot.launcher.stop();
                });
    }

    private static Command startIntaking(Robot robot) {
        return Command.build()
                .setStart(() -> {
                }).setExecute(() -> robot.intake.setBehavior(Intake.Behavior.INTAKE))
                .setDone(() -> true)
                .setEnd(endCondition -> {
                });
    }

    private static Command stopIntaking(Robot robot) {
        return Command.build()
                .setStart(() -> {
                }).setExecute(() -> robot.intake.setBehavior(Intake.Behavior.OFF))
                .setDone(() -> true)
                .setEnd(endCondition -> {
                });
    }

    @Override
    protected Command getAutoRoutine() {
        return sequential(
                follow(follower, paths.startToShoot1()),
                shoot(robot),
                follow(follower, paths.shootToPrepPickup()),
                startIntaking(robot),
                follow(follower, paths.prepPickupToPickup()),
                stopIntaking(robot),
                follow(follower, paths.pickupToBackUp()),
                follow(follower, paths.backUpToShoot2()),
                shoot(robot),
                follow(follower, paths.shoot2ToPark1()),
                follow(follower, paths.park1ToPark2()),
                follow(follower, paths.park2ToPark3())
        );
    }

    @Override
    protected void configure() {

    }


}
