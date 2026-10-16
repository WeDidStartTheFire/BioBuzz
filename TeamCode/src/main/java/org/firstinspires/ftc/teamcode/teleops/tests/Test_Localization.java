package org.firstinspires.ftc.teamcode.teleops.tests;

import static org.firstinspires.ftc.teamcode.MatchContext.Mode.TELEOP;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.PrintLevel.INFO;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MatchContext;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "Test Localization", group = "C")
public class Test_Localization extends OpMode {
    private Robot robot;
    private TelemetryUtils tm;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, new MatchContext(TELEOP, Color.BLUE));
        robot.drivetrain.setPose(loadOdometryPosition());
        tm = robot.tm();
    }

    @Override
    public void loop() {
        tm.drawRobot(robot.drivetrain.follower);
        Pose drivetrainPosition = robot.drivetrain.pose();
        if (drivetrainPosition != null)
            tm.print(drivetrainPosition, INFO);
        tm.update();
    }
}
