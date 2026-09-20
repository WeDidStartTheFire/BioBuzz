package org.firstinspires.ftc.teamcode.teleops.other;

import static org.firstinspires.ftc.teamcode.RobotState.validStartPose;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.controllers.TeleOpController;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "TeleOp_HivePositionDetector", group = "B")
public class TeleOp_HivePositionDetector extends OpMode {
    Robot robot;
    TeleOpController teleop;
    TelemetryUtils tm;

    @Override
    public void init() {
        RobotState.color = RobotConstants.Color.BLUE;
        Pose pose = loadOdometryPosition();
        RobotState.auto = false;
        validStartPose = pose != null;
        RobotState.pose = validStartPose ? pose : new Pose();
        robot = new Robot(hardwareMap, telemetry, true);
        robot.drivetrain.useLimelightFollower();
        robot.drivetrain.follower.setPose(RobotState.pose);
        robot.drivetrain.follower.startTeleopDrive();
        teleop = new TeleOpController(robot, gamepad1, gamepad2);
        tm = robot.drivetrain.tm;
    }

    @Override
    public void loop() {
        if (robot.limelight.getFiducials() != null) {
            for (LLResultTypes.FiducialResult fiducial : robot.limelight.getFiducials()) {
                tm.print(fiducial.getTargetPoseRobotSpace().toString());
            }
        }
        tm.update();
    }
}
