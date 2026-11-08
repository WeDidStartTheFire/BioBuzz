package org.firstinspires.ftc.teamcode.teleops.tests;

import static org.firstinspires.ftc.teamcode.MatchContext.Mode.TELEOP;
import static org.firstinspires.ftc.teamcode.utils.TelemetryUtils.ErrorLevel.MEDIUM;
import static org.firstinspires.ftc.teamcode.utils.TelemetryUtils.PrintLevel.INFO;
import static org.firstinspires.ftc.teamcode.utils.Utils.loadOdometryPosition;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MatchContext;
import org.firstinspires.ftc.teamcode.controllers.TeleOpController;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;

@TeleOp(name = "Test: Limelight Object Detection", group = "Test")
public class Test_Limelight_Object_Detection extends OpMode {
    private TeleOpController teleop;
    private TelemetryUtils tm;
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, new MatchContext(TELEOP, Color.BLUE));
        robot.drivetrain.setPose(loadOdometryPosition());
        teleop = new TeleOpController(robot, gamepad1, gamepad2);
        tm = robot.tm();
        if (!robot.drivetrain.isPoseValid())
            tm.warn(MEDIUM, "Robot Centric driving will be used until the position is reset");
        else tm.print("Field Centric Driving", "✅", INFO);
        tm.print("Color", "🟦🟦Blue🟦🟦", INFO);
    }

    @Override
    public void start() {
        teleop.start();
    }

    @Override
    public void loop() {
        for (double x : robot.limelight.getPositionXOfObjects()) {
            tm.print("X Position of Object", x, INFO);
        }
        for (double y : robot.limelight.getPositionYOfObjects()) {
            tm.print("Y Position of Object", y, INFO);
        }
        tm.update();
    }

    @Override
    public void stop() {
        teleop.stop();
    }
}
