package org.firstinspires.ftc.teamcode.autos.primary;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class TestAuto2 extends LinearOpMode {

    private Follower follower;

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

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.line(start, path1).heading(Interpolator.piecewise().until(0.5, Interpolator.facingPoint(path1Segment1Target)).until(1, Interpolator.linear(path1Segment2Start, path1Segment2End)));
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.curve(point2, point3Control1, point3Control2, point3).heading(Interpolator.piecewise().until(0.3567, Interpolator.linear(point3Segment1Start, point3Segment1End)).until(1, Interpolator.facingPoint(point3Segment2Target)));
    }

    public Path path4() {
        return Paths.curve(point3, point4Control1, point4Control2, point4Control3, point4).heading(Interpolator.piecewise().until(0.6373, Interpolator.facingPoint(point4Segment1Target)).until(1, Interpolator.linear(point4Segment2Start, point4Segment2End)));
    }

    public Path path5() {
        return Paths.line(point4, point5).constant(point5);
    }
}
