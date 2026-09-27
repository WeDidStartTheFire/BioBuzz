package org.firstinspires.ftc.teamcode.autos.primary;

import static java.lang.Math.toRadians;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import androidx.annotation.NonNull;

@Autonomous(name = "(Blue) 8 Points")
public class Blue_8PtAuto extends OpMode {

    // Constants.create needs to be updated after we do tuning
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(38.5909, 8.358, 90);
    private final Pose end = poseFactory.of(123.0907, 35.9239, 180);


    public Path path1() {
        return Paths.line(start, end).linear(start, end);
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
    }

    public void start() {
        schedule(follow(follower, path1()));
    }
    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }
}