package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta.Flavor.TELEOP;
import static org.firstinspires.ftc.teamcode.RobotConstants.BLUE_TELEOP_NAME;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.controllers.TeleOpController;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotRefactor;

@TeleOp(name = "TeleOp_Refactor_Testing", group = "B")
public class TeleOp_Refactor_Testing extends OpMode {
    public TeleOpController teleop;
    public RobotRefactor robot;
    public TelemetryUtils tm;

    @Override
    public void init() {
        robot = new RobotRefactor(hardwareMap, telemetry, true);
        robot.drivetrain.follower.startTeleopDrive();
        teleop = new TeleOpController(robot, gamepad1, gamepad2);
        tm = robot.drivetrain.tm;
    }

    @Override
    public void start() {
        teleop.start();
    }

    @Override
    public void loop() {
        teleop.drivetrainLogic(true);
        teleop.update();
        robot.update();
    }
}
