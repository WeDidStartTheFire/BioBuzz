package org.firstinspires.ftc.teamcode.autos.primary;

import static org.firstinspires.ftc.teamcode.TelemetryUtils.PrintLevel.INFO;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autos.BaseAuto;
import org.firstinspires.ftc.teamcode.enums.Color;

@Autonomous(name = Auto_Test2.name, group = "B")
public class Auto_Test2 extends BaseAuto<Auto_Test2.State> {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(132.125, 9.375, 90);
    private final Pose path1 = poseFactory.of(32, 40, 0);
    private final Pose path1Segment1Target = poseFactory.of(144, 144, 0);
    private final Pose path1Segment2Start = poseFactory.of(32, 40, 62.5653);
    private final Pose path1Segment2End = poseFactory.of(32, 40, 0);
    private final Pose point2Start = poseFactory.of(32, 40, 0);
    private final Pose point2 = poseFactory.of(32, 108, 60);
    private final Pose point3 = poseFactory.of(108.5, 32.5, -163.325);
    private final Pose point3Control1 = poseFactory.of(170, 175, 0);
    private final Pose point3Control2 = poseFactory.of(96, 56, 0);
    private final Pose point3Segment1Start = poseFactory.of(108.5, 32.5, 60);
    private final Pose point3Segment1End = poseFactory.of(108.5, 32.5, -136);
    private final Pose point3Segment2Target = poseFactory.of(0, 0, 0);
    private final Pose point4 = poseFactory.of(58.2378, 13.1292, 0);
    private final Pose point4Control1 = poseFactory.of(77.9609, 55.4343, 0);
    private final Pose point4Control2 = poseFactory.of(133.9934, 4.0753, 0);
    private final Pose point4Control3 = poseFactory.of(134.1824, 70.4505, 0);
    private final Pose point4Segment1Target = poseFactory.of(0, 0, 0);
    private final Pose point4Segment2Start = poseFactory.of(58.2378, 13.1292, -166);
    private final Pose point4Segment2End = poseFactory.of(58.2378, 13.1292, 0);
    private final Pose point5 = poseFactory.of(33, 33, 0);

    private Path pa1, pa2, pa3, pa4, pa5;

    public static final String name = "Test Auto 2";
    private final Color color = Color.BLUE;
    private final State initialState = State.START;

    protected enum State {
        START, PATH1, PATH2, PATH3, PATH4, PATH5, END
    }

    @Override
    protected void buildPaths() {
        pa1 = Paths.line(start, path1).heading(Interpolator.piecewise().until(0.5,
                Interpolator.facingPoint(path1Segment1Target)).until(1,
                Interpolator.linear(path1Segment2Start, path1Segment2End)));
        pa2 = Paths.line(point2Start, point2).linear(point2Start, point2);
        pa3 = Paths.curve(point2, point3Control1, point3Control2, point3).heading(
                Interpolator.piecewise().until(0.3567,
                        Interpolator.linear(point3Segment1Start, point3Segment1End)).until(1,
                        Interpolator.facingPoint(point3Segment2Target)));
        pa4 = Paths.curve(point3, point4Control1, point4Control2, point4Control3, point4).heading(
                Interpolator.piecewise().until(0.6373,
                        Interpolator.facingPoint(point4Segment1Target)).until(1,
                        Interpolator.linear(point4Segment2Start, point4Segment2End)));
        pa5 = Paths.line(point4, point5).constant(point5);
    }

    @Override
    protected void configure() {
        super.configure(start, initialState, color, name);
    }

    @Override
    protected void pathUpdate() {
        switch (state) {
            case START:
                robot.drivetrain.follower.follow(pa1);
                setState(State.PATH1);
                break;
            case PATH1:
                if (robot.drivetrain.follower.isBusy()) break;
                robot.drivetrain.follower.follow(pa2);
                setState(State.PATH2);
                break;
            case PATH2:
                if (robot.drivetrain.follower.isBusy()) break;
                robot.drivetrain.follower.follow(pa3);
                setState(State.PATH3);
                break;
            case PATH3:
                if (robot.drivetrain.follower.isBusy()) break;
                robot.drivetrain.follower.follow(pa4);
                setState(State.PATH4);
                break;
            case PATH4:
                if (robot.drivetrain.follower.isBusy()) break;
                robot.drivetrain.follower.follow(pa5);
                setState(State.PATH5);
                break;
            case PATH5:
                if (robot.drivetrain.follower.isBusy()) break;
                setState(State.END);
                break;
            case END:
                tm.print("Finished", INFO);
                break;
        }
    }
}
