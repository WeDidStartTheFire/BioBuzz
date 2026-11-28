package org.firstinspires.ftc.teamcode.autos.primary;

import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;
import com.pedropathing.utils.Timer;

import org.firstinspires.ftc.teamcode.autos.BaseAuto2;
import org.firstinspires.ftc.teamcode.enums.Color;

public class Auto_Test_Deadline extends BaseAuto2<org.firstinspires.ftc.teamcode.autos.paths.Paths_Red> {

    public static final String baseName = "Deadline";
    public static final String group = "A";
    public Timer timer = new Timer();

    @Override
    protected void configure() {
        configure(org.firstinspires.ftc.teamcode.autos.paths.Paths_Red.class, Color.RED, "Test Deadline");
    }
    private static Command wait(int time, Timer timer) {
        return Command.build()
                .setStart(timer::reset)
                .setDone(() -> timer.seconds() >= time);
    }

    private Command followPaths() {
        return sequential(
                follow(follower, paths.startToShoot1()),
                autoStates.setCurrentState("SHOOT"),
                wait(3, timer),
                autoStates.setCurrentState("IDLE"),
                follow(follower, paths.shoot1ToPickup()),
                autoStates.setCurrentState("INTAKE"),
                wait(1, timer),
                follow(follower, paths.pickupToShoot2()),
                autoStates.setCurrentState("SHOOT"),
                wait(3, timer),
                autoStates.setCurrentState("IDLE"),
                follow(follower, paths.shootToFinish())
        );
    }

    @Override
    protected Command getAutoRoutine() {
        return deadline(
                deadline(
                        followPaths(), // Deadline, everything will stop when this is finished.
                        autoStates.update()
                )
        );
    }
}


