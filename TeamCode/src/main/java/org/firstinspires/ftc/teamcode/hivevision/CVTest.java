package org.firstinspires.ftc.teamcode.hivevision;

import static org.firstinspires.ftc.teamcode.utils.TelemetryUtils.PrintLevel.INFO;
import static org.firstinspires.ftc.teamcode.utils.TelemetryUtils.PrintLevel.VERBOSE;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;

import java.util.ArrayList;


@TeleOp
public class CVTest extends LinearOpMode {
    private CVPathGen pather;
    private Follower follower;
    public static boolean isRed = true;
    ArrayList<Pose> allBalls = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(35, 8.5, Math.toRadians(90)));
        follower.update();
        pather = new CVPathGen(hardwareMap, follower, isRed, telemetry);
        TelemetryUtils tm = new TelemetryUtils(telemetry);

        waitForStart();
        while (opModeIsActive()) {
            follower.update();
            tm.drawRobot(follower);
            allBalls = pather.getBalls(false);
            if (gamepad1.aWasPressed()) {
                Path path = pather.pathGen();
                if (path != null) {
                    tm.print(path, INFO);
                    follower.follow(path);
                }
            }
            if (follower.isBusy() && follower.atParametricEnd()) {
                follower.stop();
            }
            for (Pose ball : allBalls) {
                telemetry.addLine("(" + ball.x() + ", " + ball.y() + ")");
            }
            tm.print("Robot X", follower.pose().x(), VERBOSE);
            tm.print("Robot Y", follower.pose().y(), VERBOSE);
            tm.print("Remaining", follower.remainingDistance(), VERBOSE);
            tm.print("Dist to endpoint", follower.distanceToEndpoint(), VERBOSE);
            tm.print("Completion", follower.completion(), VERBOSE);
            tm.update();
        }
    }
}
