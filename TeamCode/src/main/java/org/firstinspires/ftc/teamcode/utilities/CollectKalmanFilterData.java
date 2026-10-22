package org.firstinspires.ftc.teamcode.utilities;

import static org.firstinspires.ftc.teamcode.MatchContext.Mode.TELEOP;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.ErrorLevel.CRITICAL;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.ErrorLevel.MEDIUM;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.PrintLevel.INFO;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.PrintLevel.VERBOSE;
import static org.firstinspires.ftc.teamcode.Utils.addLine;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MatchContext;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.controllers.TeleOpController;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.enums.RobotEnvironment;
import org.firstinspires.ftc.teamcode.pedro.localizers.KalmanLocalizer;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;

@TeleOp(name = "Kalman Test", group = "Test")
public class CollectKalmanFilterData extends OpMode {
    private TeleOpController teleop;
    private Robot robot;
    private TelemetryUtils tm;
    private FileWriter fw;
    private BufferedWriter bw;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, new MatchContext(TELEOP, Color.BLUE));
        robot.drivetrain.useKalmanFollower();
        robot.drivetrain.setPose(loadOdometryPosition());
        teleop = new TeleOpController(robot, gamepad1, gamepad2);
        tm = robot.tm();
        tm.setEnvironment(RobotEnvironment.DEBUG);
        ((KalmanLocalizer) robot.drivetrain.follower.localizer).enableTelemetry(tm);
        if (!robot.drivetrain.isPoseValid())
            tm.warn(MEDIUM, "Robot Centric driving will be used until the position is reset");
        else tm.print("Field Centric Driving", "✅", INFO);
        tm.print("Color", "🟦🟦Blue🟦🟦", INFO);

        File dir = new File(hardwareMap.appContext.getFilesDir().toURI());
        File file = new File(dir, "kalman_data_0.csv");
        int i = 0;
        while (true) {
            try {
                if (file.createNewFile()) break;
            } catch (IOException e) {
                tm.warn(CRITICAL, Arrays.toString(e.getStackTrace()));
                tm.update();
                throw new RuntimeException(e);
            }
            i++;
            file = new File(dir, "kalman_data_" + i + ".csv");
        }
        try {
            fw = new FileWriter(file, true);
            bw = new BufferedWriter(fw);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void init_loop() {
        Pose pose = robot.drivetrain.pose();
        if (pose != null) tm.print(pose, INFO);
        teleop.update();
    }

    @Override
    public void start() {
        teleop.start();
    }

    @Override
    public void loop() {
        robot.drivetrain.update();
        LLResult result = robot.limelight.getLatestResult();
        tm.drawRobot(robot.drivetrain.follower);
        if (result != null) {
            tm.print("LL Pose MT1", result.getBotpose(), VERBOSE);
            tm.print("LL Std Dev MT1 X", result.getStddevMt1()[0], VERBOSE);
            tm.print("LL Std Dev MT1 Y", result.getStddevMt1()[1], VERBOSE);
            tm.print("LL Std Dev MT1 Heading", result.getStddevMt1()[5], VERBOSE);
            tm.print("====================", VERBOSE);
            tm.print("LL Pose MT2", result.getBotpose_MT2(), VERBOSE);
            tm.print("LL Std Dev MT2 X", result.getStddevMt2()[0], VERBOSE);
            tm.print("LL Std Dev MT2 Y", result.getStddevMt2()[1], VERBOSE);
            tm.print("LL Std Dev MT2 Heading", result.getStddevMt2()[5], VERBOSE);
        }
        addLine(bw, ((KalmanLocalizer) robot.drivetrain.follower.localizer).getLog());
        teleop.update();
    }


    @Override
    public void stop() {
        teleop.stop();
        try {
            bw.close();
            fw.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
