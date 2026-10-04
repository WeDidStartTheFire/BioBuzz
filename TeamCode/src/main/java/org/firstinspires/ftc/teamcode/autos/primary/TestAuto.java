package org.firstinspires.ftc.teamcode.autos.primary;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autos.BaseAuto;
import org.firstinspires.ftc.teamcode.enums.Color;

@Autonomous(name = TestAuto.name)
public class TestAuto extends BaseAuto<TestAuto.State> {
    private final TestAuto.State initialState = TestAuto.State.START;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private Path pa1, pa2, pa3, pa4;
    private final Color color = Color.BLUE;
    public static final String name = "Tuning Test";
    protected enum State {
        FINISHED,
        START,
        P2,
        P3,
        P4
    }

    private final Pose start = poseFactory.of(132.682, 8.2894, 0);
    private final Pose pt2 = poseFactory.of(129.7883, 122.2311, 0);
    private final Pose pt3 = poseFactory.of(21.5184, 121.1493, 0);
    private final Pose pt4 = poseFactory.of(22.1564, 22.7566, 0);
    private final Pose end = poseFactory.of(116.682, 8.2894, 0);


    @Override
    protected void buildPaths() {
        pa1 = Paths.line(start, pt2).linear(start, pt2);
        pa2 = Paths.line(pt2, pt3).linear(pt2, pt3);
        pa3 = Paths.line(pt3, pt4).linear(pt3, pt4);
        pa4 = Paths.line(pt4, end).linear(pt4, end);
    }

    @Override
    protected void pathUpdate() {
        robot.drivetrain.follower.update();
        switch (state) {
            case START:
                robot.drivetrain.follower.follow(pa1);
                setState(TestAuto.State.P2);
                break;
            case P2:
                if (robot.drivetrain.follower.isBusy()) break;
                robot.drivetrain.follower.follow(pa2);
                setState(TestAuto.State.P3);
                break;
            case P3:
                if (robot.drivetrain.follower.isBusy()) break;
                robot.drivetrain.follower.follow(pa3);
                setState(TestAuto.State.P4);
                break;
            case P4:
                if (robot.drivetrain.follower.isBusy()) break;
                robot.drivetrain.follower.follow(pa4);
                setState(State.FINISHED);
                break;
            case FINISHED:
                break;
        }
    }
    @Override
    protected void configure() {
        configure(startPose, initialState, color, name);
    }
}
