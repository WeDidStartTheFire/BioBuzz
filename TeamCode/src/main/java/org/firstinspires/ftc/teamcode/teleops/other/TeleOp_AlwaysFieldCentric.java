package org.firstinspires.ftc.teamcode.teleops.other;

import static org.firstinspires.ftc.teamcode.MatchContext.Mode.TELEOP;
import static org.firstinspires.ftc.teamcode.utils.TelemetryUtils.PrintLevel.INFO;
import static org.firstinspires.ftc.teamcode.utils.Utils.loadOdometryPosition;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MatchContext;
import org.firstinspires.ftc.teamcode.controllers.TeleOpController;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;

@TeleOp(name = "Always Field Centric", group = "C")
public class TeleOp_AlwaysFieldCentric extends OpMode {
    private TeleOpController teleop;

    @Override
    public void init() {
        Robot robot = new Robot(hardwareMap, telemetry, new MatchContext(TELEOP, Color.BLUE));
        robot.drivetrain.setPose(loadOdometryPosition());
        teleop = new TeleOpController(robot, gamepad1, gamepad2);
        TelemetryUtils tm = robot.tm();
        if (!robot.drivetrain.isPoseValid())
            tm.print("Field Centric Driving️", "☑️Will be used without valid position", INFO);
        else tm.print("Field Centric Driving", "✅", INFO);
        tm.print("Color", "\uD83D\uDFE6Blue\uD83D\uDFE6 (Default)", INFO);
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
