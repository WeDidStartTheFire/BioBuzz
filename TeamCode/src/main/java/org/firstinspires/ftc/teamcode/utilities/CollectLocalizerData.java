package org.firstinspires.ftc.teamcode.utilities;

import static org.firstinspires.ftc.teamcode.MatchContext.Mode.TELEOP;
import static org.firstinspires.ftc.teamcode.utils.TelemetryUtils.ErrorLevel.CRITICAL;
import static org.firstinspires.ftc.teamcode.utils.TelemetryUtils.ErrorLevel.MEDIUM;
import static org.firstinspires.ftc.teamcode.utils.TelemetryUtils.PrintLevel.INFO;
import static org.firstinspires.ftc.teamcode.utils.TelemetryUtils.PrintLevel.VERBOSE;
import static org.firstinspires.ftc.teamcode.utils.Utils.addLine;
import static org.firstinspires.ftc.teamcode.utils.Utils.loadOdometryPosition;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MatchContext;
import org.firstinspires.ftc.teamcode.controllers.TeleOpController;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.enums.RobotEnvironment;
import org.firstinspires.ftc.teamcode.pedro.localizers.LoggingLocalizer;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;

@TeleOp(name = "Collect Localizer Data", group = "Test")
public class CollectLocalizerData extends OpMode {
    private TeleOpController teleop;
    private Robot robot;
    private TelemetryUtils tm;
    private FileWriter fw;
    private BufferedWriter bw;
    private String followerType = "kalman";

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, new MatchContext(TELEOP, Color.BLUE));
        teleop = new TeleOpController(robot, gamepad1, gamepad2);
        tm = robot.tm();
        tm.setEnvironment(RobotEnvironment.DEBUG);
        if (!robot.drivetrain.isPoseValid())
            tm.warn(MEDIUM, "Robot Centric driving will be used until the position is reset");
        else tm.print("Field Centric Driving", "✅", INFO);
        tm.print("Color", "🟦🟦Blue🟦🟦", INFO);
    }

    @Override
    public void init_loop() {
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            followerType = "kalman";
        }
        if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
            followerType = "complementary";
        }
        if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
            followerType = "limelight";
        }
        tm.print("Localizer Type", followerType, INFO);
        tm.print("", INFO);
        tm.print("Options:", INFO);
        tm.print("A", "kalman", INFO);
        tm.print("B", "complementary", INFO);
        tm.print("X", "limelight", INFO);
        tm.updateOnlyPanels();
    }

    @Override
    public void start() {
        switch (followerType) {
            case "complementary":
                robot.drivetrain.useComplementaryFollower();
                break;
            case "limelight":
                robot.drivetrain.useLimelightFollower();
                break;
            case "kalman":
            default:
                robot.drivetrain.useKalmanFollower();
                break;
        }
        robot.drivetrain.setPose(loadOdometryPosition());
        ((LoggingLocalizer) robot.drivetrain.follower.localizer).enableTelemetry(tm);
        ((LoggingLocalizer) robot.drivetrain.follower.localizer).initializeLogging();
        File dir = new File(hardwareMap.appContext.getFilesDir().toURI());
        File file = new File(dir, followerType + "_data_0.csv");
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
            file = new File(dir, followerType + "_data_" + i + ".csv");
        }
        try {
            fw = new FileWriter(file, true);
            bw = new BufferedWriter(fw);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        addLine(bw, ((LoggingLocalizer) robot.drivetrain.follower.localizer).getHeader());
        tm.drawRobot(robot.drivetrain.follower);
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
        addLine(bw, ((LoggingLocalizer) robot.drivetrain.follower.localizer).getLog());
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
