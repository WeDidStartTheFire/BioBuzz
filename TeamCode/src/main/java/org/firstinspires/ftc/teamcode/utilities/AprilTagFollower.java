package org.firstinspires.ftc.teamcode.utilities;

import static org.firstinspires.ftc.teamcode.MatchContext.Mode.TELEOP;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MatchContext;
import org.firstinspires.ftc.teamcode.controllers.TeleOpController;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Utility(name = "AprilTag Follower", description = "Follows AprilTags")
public class AprilTagFollower extends OpMode {
    private Robot robot;
    private TeleOpController teleop;
    private Gamepad gamepad;
    private int targetID = 0;

    @Override
    public void init() {
        gamepad = new Gamepad();
        gamepad.left_stick_x = 0;
        robot = new Robot(hardwareMap, telemetry, new MatchContext(TELEOP, Color.BLUE));
        robot.drivetrain.useLimelightFollower();
        robot.drivetrain.setPose(loadOdometryPosition());
        teleop = new TeleOpController(robot, gamepad, gamepad2);
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
        if (robot.limelight.getFiducials() != null) {
            for (LLResultTypes.FiducialResult fiducial : robot.limelight.getFiducials()) {
                if (targetID == 0) {
                    targetID = fiducial.getFiducialId();
                } if (fiducial.getFiducialId() == targetID) {
                    if (fiducial.getTargetXPixels() <= -0.5) { gamepad.left_stick_x = -0.25f; }
                    else if (fiducial.getTargetXPixels() >= 0.5) { gamepad.left_stick_x = 0.25f; }
                }
            }
        }
        teleop.update();
    }
}
