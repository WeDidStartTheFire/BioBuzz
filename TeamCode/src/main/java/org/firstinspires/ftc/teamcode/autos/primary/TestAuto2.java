package org.firstinspires.ftc.teamcode.autos.primary;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.autos.BaseAuto;
import org.firstinspires.ftc.teamcode.enums.Color;

@Autonomous (name= TestAuto2.name)
public class TestAuto2 extends BaseAuto <TestAuto2.State> {
    public final static String name = "Test Auto 2";
    private final Color color = Color.BLUE;
    private final State initialState = State.START;
    protected enum State {
        START,
        FOLLOWING_PATH1,
        FOLLOWING_PATH2,
        FOLLOWING_PATH3,
        FOLLOWING_PATH4,
        FINISHED

    }
    private final Pose start = new Pose(132.682, 8.2894, 0);
    private final Pose pt2 = new Pose(129.7883, 122.2311, 0);
    private final Pose pt3 = new Pose(21.5184, 121.1493, 0);
    private final Pose pt4 = new Pose(22.1564, 22.7566, 0);
    private final Pose end = new Pose(116.682, 8.2894, 0);

    private PathChain path1, path2, path3, path4;
    @Override
    protected void buildPaths() {
        path1 = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(start, pt2))
                .setLinearHeadingInterpolation(start.getHeading(), pt2.getHeading())
                .build();
        path2 = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(pt2, pt3))
                .setLinearHeadingInterpolation(pt2.getHeading(), pt3.getHeading())
                .build();
        path3 = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(pt3, pt4))
                .setLinearHeadingInterpolation(pt3.getHeading(), pt4.getHeading())
                .build();
        path4 = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(pt4, end))
                .setLinearHeadingInterpolation(pt4.getHeading(), end.getHeading())
                .build();
    }

    @Override
    protected void pathUpdate() {
        switch (state) {
            case START:
                robot.drivetrain.follower.followPath(path1);
                setState(State.FOLLOWING_PATH1);
                break;
            case FOLLOWING_PATH1:
                if (!robot.drivetrain.follower.isBusy()) {
                    robot.drivetrain.follower.followPath(path2);
                    setState(State.FOLLOWING_PATH2);
                }
                break;
            case FOLLOWING_PATH2:
                if (!robot.drivetrain.follower.isBusy()) {
                    robot.drivetrain.follower.followPath(path3);
                    setState(State.FOLLOWING_PATH3);
                }
                break;
            case FOLLOWING_PATH3:
                if (!robot.drivetrain.follower.isBusy()) {
                    robot.drivetrain.follower.followPath(path4);
                    setState(State.FOLLOWING_PATH4);
                }
                break;
            case FOLLOWING_PATH4:
                if (!robot.drivetrain.follower.isBusy()) {
                    setState(State.FINISHED);
                }
                break;
            case FINISHED:
                tm.print("Finished following the path sequence.", TelemetryUtils.PrintLevel.INFO);
                break;
        }
    }

    @Override
    protected void configure() {
        super.startPose = start;
        super.color = color;
        super.initialState = initialState;
        super.name = name;
    }
}
