package org.firstinspires.ftc.teamcode.autos.primary;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autos.BaseAuto;
import org.firstinspires.ftc.teamcode.enums.Color;

@Autonomous(name = Blue_8PtAuto.name)
public class Blue_8PtAuto extends BaseAuto<Blue_8PtAuto.State> {
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(38.5909, 8.358, 90);
    private final Pose end = poseFactory.of(123.0907, 35.9239, 180);
    private Path path1;

    static final String name = "🟦Blue🟦 8 Points";
    static final Color color = Color.BLUE;
    private final State initialState = State.NONE;

    protected enum State {
        NONE
    }


    @Override
    protected void buildPaths() {
        path1 = Paths.line(start, end).linear(start, end);
    }

    @Override
    protected void onStart() {
        schedule(follow(robot.drivetrain.follower, path1));
    }

    @Override
    protected void pathUpdate() {
        Scheduler.execute();
    }

    @Override
    protected void configure() {
        super.configure(startPose, initialState, color, name);
    }
}
