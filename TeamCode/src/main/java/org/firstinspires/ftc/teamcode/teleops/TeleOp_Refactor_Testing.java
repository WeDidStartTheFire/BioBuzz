package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MatchContext;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.controllers.TeleOpController;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "TeleOp_Refactor_Testing", group = "B")
public class TeleOp_Refactor_Testing extends OpMode {
    public TeleOpController teleop;
    public Robot robot;
    public TelemetryUtils tm;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, new MatchContext(MatchContext.Mode.TELEOP, Color.BLUE));
        teleop = new TeleOpController(robot, gamepad1, gamepad2);
        tm = robot.tm();
    }

    @Override
    public void start() {
        teleop.start();
    }

    @Override
    public void loop() {
        teleop.drivetrainLogic(true);
        teleop.update();
    }
}
