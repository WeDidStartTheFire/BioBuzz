package org.firstinspires.ftc.teamcode.autos.primary;

import static java.lang.Math.toRadians;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autos.BaseAuto;
import org.firstinspires.ftc.teamcode.enums.Color;

@Autonomous(name = "(Red) 8 Points")
public class Red_8PtAuto extends BaseAuto<Red_8PtAuto.State> {
    private final State initialState = State.START_TO_PARK;
    private PathChain startToPark;
    protected enum State {
        FINISHED,
        START_TO_PARK
    }
    private final Pose start = new Pose(103.1667, 133.4205, toRadians(90));
    private final Pose end = new Pose(14.2858, 101.3141, toRadians(180));


    @Override
    protected void buildPaths() {
        startToPark = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    @Override
    protected void pathUpdate() {
        robot.drivetrain.follower.update();
        switch (state) {
            case START_TO_PARK:
                robot.drivetrain.follower.followPath(startToPark, true);
                setState(State.FINISHED);
                break;
            case FINISHED:
                break;
        }
    }
    @Override
    protected void configure() {
        super.startPose = start;
        super.color = Color.RED;
        super.initialState = initialState;
    }
}
