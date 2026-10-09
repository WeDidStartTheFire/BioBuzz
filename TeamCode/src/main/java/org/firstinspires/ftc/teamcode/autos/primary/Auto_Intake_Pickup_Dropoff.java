package org.firstinspires.ftc.teamcode.autos.primary;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autos.BaseAuto;
import org.firstinspires.ftc.teamcode.enums.Color;

@Autonomous(name = Auto_Intake_Pickup_Dropoff.name, group = "B")
public class Auto_Intake_Pickup_Dropoff extends BaseAuto<Auto_Intake_Pickup_Dropoff.State> {
    private final Auto_Intake_Pickup_Dropoff.State initialState = Auto_Intake_Pickup_Dropoff.State.START;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private Path pa1, pa2, pa3, pa3Control1, pa4, pa5, pa6, pa7, pa8;
    private final Color color = Color.BLUE;
    public static final String name = "Tuning Test";

    protected enum State {
        FINISHED,
        START,
        P2,
        P3,
        P4,
        P5,
        P6,
        P6OUTTAKE,
        P7,
        P8
    }


    private final Pose start = poseFactory.of(35.1125, 8.6368, 90);
    private final Pose path1 = poseFactory.of(35.1125, 63.5104, 90);
    private final Pose point2 = poseFactory.of(35.1035, 96.7363, 90.0155);
    private final Pose point3 = poseFactory.of(70.5243, 96.5243, -68.9548);
    private final Pose point3Control1 = poseFactory.of(54.7543, 137.9739, 0);
    private final Pose point4 = poseFactory.of(70.8578, 46.4955, 90.3819);
    private final Pose point5 = poseFactory.of(117.7223, 47.2408, 62.09);
    private final Pose point5Control1 = poseFactory.of(94.0666, 2.1386, 0);
    private final Pose point6 = poseFactory.of(119.8924, 126.6962, 85.451);
    private final Pose point6Control1 = poseFactory.of(117.0716, 91.5045, 0);
    private final Pose point7 = poseFactory.of(35.6071, 95.8866, -94.2547);
    private final Pose point7Control1 = poseFactory.of(110.2324, 109.048, 0);
    private final Pose point7Control2 = poseFactory.of(38.1652, 140.3083, 0);
    private final Pose point8 = poseFactory.of(35.0639, 15.8947, 89.6109);

    @Override
    protected void buildPaths() {
        pa1 = Paths.line(start, path1).tangent();
        pa2 = Paths.line(path1, point2).tangent();
        pa3 = Paths.curve(point2, point3Control1, point3).tangent();
        pa4 = Paths.line(point3, point4).reverseTangent();
        pa5 = Paths.curve(point4, point5Control1, point5).tangent();
        pa6 = Paths.curve(point5, point6Control1, point6).tangent();
        pa7 = Paths.curve(point6, point7Control1, point7Control2, point7).tangent();
        pa8 = Paths.line(point7, point8).reverseTangent();
    }

    @Override
    protected void pathUpdate() {
        robot.drivetrain.follower.update();
        switch (state) {
            case START:
                robot.drivetrain.follower.follow(pa1);
                setState(State.P2);
                break;
            case P2:
                if (robot.drivetrain.follower.isBusy()) break;
                robot.drivetrain.follower.follow(pa2);
                setState(State.P3);
                break;
            case P3:
                if (robot.drivetrain.follower.isBusy()) break;
                robot.drivetrain.follower.follow(pa3);
                setState(State.P4);
                break;
            case P4:
                if (robot.drivetrain.follower.isBusy()) break;
                robot.drivetrain.follower.follow(pa4);
                setState(State.P5);
                break;
            case P5:
                if (robot.drivetrain.follower.isBusy()) break;
                robot.drivetrain.follower.follow(pa5);
                setState(State.P6);
                break;
            case P6:
                if (robot.drivetrain.follower.isBusy()) break;
                robot.drivetrain.follower.follow(pa6);
                setState(State.FINISHED);
                break;
            case P6OUTTAKE:
                if (robot.drivetrain.follower.isBusy()) break;
                intakeController.outtake();

                setState(State.P8);
                break;
            case FINISHED:
                intakeController.stop();
                break;
        }
    }

    @Override
    protected void configure() {
        configure(startPose, initialState, color, name);
    }
}
