package org.firstinspires.ftc.teamcode.teleops.tests;

import static org.firstinspires.ftc.teamcode.MatchContext.Mode.TELEOP;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.PrintLevel.VERBOSE;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MatchContext;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.controllers.TeleOpController;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "Limelight Test", group = "Test")
@Disabled
public class Test_Limelight extends OpMode {
    public TeleOpController teleop;
    public Robot robot;
    public TelemetryUtils tm;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, new MatchContext(TELEOP, Color.BLUE));
        robot.drivetrain.useLimelightFollower();
        robot.drivetrain.setPose(loadOdometryPosition());
        teleop = new TeleOpController(robot, gamepad1, gamepad2);
        tm = robot.drivetrain.tm;
    }

    @Override
    public void init_loop() {
        teleop.update();
        Pose pose = robot.drivetrain.pose();
        if (pose != null) tm.print(pose, VERBOSE);
    }

    @Override
    public void start() {
        teleop.start();
    }

    @Override
    public void loop() {
        teleop.drivetrainLogic();
        teleop.updateIntake();
        teleop.updateLauncherTeleOp();
        LLResult result = robot.limelight.getLatestResult();
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
        teleop.update();
    }
}
