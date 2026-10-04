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

@Autonomous(name = "(Red) 8 Points")
public class Red_8PtAuto extends BaseAuto<Red_8PtAuto.State> {
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(103.1667, 133.4205, 90);
    private final Pose end = poseFactory.of(14.2858, 101.3141, 180);
    private Path path1;

    protected enum State {
        NONE
    }

    private final State initialState = State.NONE;


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
        super.startPose = start;
        super.color = Color.BLUE;
        super.initialState = initialState;
    }
}