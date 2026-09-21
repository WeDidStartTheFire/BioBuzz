package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.RobotState.validStartPose;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.controllers.TeleOpController;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "TeleOp_AprilTag_Follower", group = "C")
public class TeleOp_AprilTag_Follower extends OpMode {

    private Robot robot;
    Limelight3A limelight;
    private TelemetryUtils tm;
    private TeleOpController teleop;
    Gamepad gamepad;
    int targetID = 0;

    @Override
    public void init() {
        gamepad = new Gamepad();
        gamepad.left_stick_x = 0;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        RobotState.color = RobotConstants.Color.BLUE;
        RobotState.auto = false;
        robot = new Robot(hardwareMap, telemetry, false);
        teleop = new TeleOpController(robot, gamepad, gamepad2);
        limelight.start();
        robot.drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tm = robot.drivetrain.tm;
        if (!validStartPose) tm.print("⚠️WARNING⚠️", "Robot Centric driving will be used");
        else tm.print("Field Centric Driving", "✅");
    }

    @Override
    public void init_loop() {
        LLResult result = limelight.getLatestResult();
        for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
            tm.print(String.valueOf(fiducial.getTargetXPixels()));
        }
        tm.update();
    }

    // Main problem, not updating Joystick values
    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        robot.initBulkCache();
        robot.updateBulkCache();
        teleop.drivetrainLogic(false, false, 1);
        for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
            if (targetID == 0) {
                targetID = fiducial.getFiducialId();
            } if (fiducial.getFiducialId() == targetID) { // Could be problem, it could be detecting different AprilTag than the one it saw First
                tm.print(String.valueOf(fiducial.getTargetXPixels()));
                if (gamepad == null) { tm.print("Gamepad returning null!"); }
                // Target pixels may be the wrong value, fine-tuning might help.
                else if (fiducial.getTargetXPixels() <= 600) { gamepad.left_stick_x = -1f; }
                else if (fiducial.getTargetXPixels() >= 900) { gamepad.left_stick_x = 1f; }
                else { gamepad.left_stick_x = 0f; } // This is here for testing, I will probably remove this.
            }
        }
        // Following prints are for reading the target ID (ID it saw first), left joystick X, and just checking if gamepad exists.
        tm.print(String.valueOf(targetID));
        tm.print(String.valueOf(gamepad.left_stick_x));
        tm.print(String.valueOf(gamepad));
        tm.update();


    }
}