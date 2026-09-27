package org.firstinspires.ftc.teamcode.autos.primary;

import static java.lang.Math.toRadians;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autos.BaseAuto;
import org.firstinspires.ftc.teamcode.enums.Color;

@Autonomous(name="Tuning Test")
public class TestAuto extends BaseAuto<TestAuto.State> {
    private final TestAuto.State initialState = TestAuto.State.START;
    private PathChain pa1, pa2, pa3, pa4;
    protected enum State {
        FINISHED,
        START,
        P2,
        P3,
        P4
    }
    private final Pose start = new Pose(132.682, 8.2894, 0);
    private final Pose pt2 = new Pose(129.7883, 122.2311, 0);
    private final Pose pt3 = new Pose(21.5184, 121.1493, 0);
    private final Pose pt4 = new Pose(22.1564, 22.7566, 0);
    private final Pose end = new Pose(116.682, 8.2894, 0);


    @Override
    protected void buildPaths() {
        pa1 = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(start, pt2))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
        pa2 = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(pt2, pt3))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
        pa3 = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(pt3, pt4))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
        pa4 = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(pt4, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    @Override
    protected void pathUpdate() {
        robot.drivetrain.follower.update();
        switch (state) {
            case START:
                robot.drivetrain.follower.followPath(pa1, true);
                setState(TestAuto.State.P2);
                break;
            case P2:
                robot.drivetrain.follower.followPath(pa2, true);
                setState(TestAuto.State.P3);
                break;
            case P3:
                robot.drivetrain.follower.followPath(pa3, true);
                setState(TestAuto.State.P4);
                break;
            case P4:
                robot.drivetrain.follower.followPath(pa4, true);
                setState(State.FINISHED);
                break;
            case FINISHED:
                break;
        }
    }
    @Override
    protected void configure() {
        super.startPose = start;
        super.color = Color.BLUE;
        super.initialState = initialState;
    }
}
