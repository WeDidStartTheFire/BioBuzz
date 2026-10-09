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
    private Path pa1, pa2, pa3, pa4, pa4Control1, pa4Control2, pa5, pa6, pa6Control1;
    private final Color color = Color.BLUE;
    public static final String name = "Tuning Test";
    protected enum State {
        FINISHED,
        START,
        P2,
        P3,
        P4,
        P5,
        P6
    }


    private final Pose start = poseFactory.of(32.3105, 8.7642, 90);
    private final Pose path1 = poseFactory.of(23.5225, 106.1769, 0);
    private final Pose point2 = poseFactory.of(67.2826, 105.7453, -0.5651);
    private final Pose point3 = poseFactory.of(94.2178, 105.7939, 0.1034);
    private final Pose point4 = poseFactory.of(94.4248, 23.1125, -156.7121);
    private final Pose point4Control1 = poseFactory.of(140.0783, 98.3263, 0);
    private final Pose point4Control2 = poseFactory.of(137.2039, 40.9613, 0);
    private final Pose point5 = poseFactory.of(81.968, 45.5923, 118.9923);
    private final Pose point6 = poseFactory.of(30.838, 29.8371, -16.3275);
    private final Pose point6Control1 = poseFactory.of(62.2646, 20.4757, 0);


    @Override
    protected void buildPaths() {
        pa1 = Paths.line(start, path1).linear(start, path1);
        pa2 = Paths.line(path1, point2).linear(path1, point2);
        pa3 = Paths.line(point2, point3).linear(point2, point3);
        pa4 = Paths.curve(point3, point4Control1, point4Control2, point4).tangent();
        pa5 = Paths.line(point4, point5).linear(point4, point5);
        pa6 = Paths.curve(point5, point6Control1, point6).tangent();
    }

    @Override
    protected void pathUpdate() {
        robot.drivetrain.follower.update();
        switch (state) {
            case START:
                robot.drivetrain.follower.follow(pa1);
                setState(Auto_Intake_Pickup_Dropoff.State.P2);
                break;
            case P2:
                if (robot.drivetrain.follower.isBusy()) break;
                robot.drivetrain.follower.follow(pa2);
                setState(Auto_Intake_Pickup_Dropoff.State.P3);
                break;
            case P3:
                if (robot.drivetrain.follower.isBusy()) break;
                intakeController.intake();
                robot.drivetrain.follower.follow(pa3);
                setState(Auto_Intake_Pickup_Dropoff.State.P4);
                break;
            case P4:
                if (robot.drivetrain.follower.isBusy()) break;
                intakeController.stop();
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
                intakeController.outtake();
                robot.drivetrain.follower.follow(pa6);
                setState(State.FINISHED);
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
