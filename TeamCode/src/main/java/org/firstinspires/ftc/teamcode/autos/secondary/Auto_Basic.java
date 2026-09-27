package org.firstinspires.ftc.teamcode.autos.secondary;

import static org.firstinspires.ftc.teamcode.MatchContext.Mode.AUTO;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MatchContext;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.enums.Dir;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name = "Basic", group = "B", preselectTeleOp = "Main")
public class Auto_Basic extends LinearOpMode {
    public Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry, new MatchContext(AUTO, Color.BLUE));
        robot.drivetrain.follower.setPose(new Pose());
        waitForStart();

        robot.drivetrain.drive(15, Dir.FORWARD, this);
        robot.drivetrain.follower.update();
        Utils.saveOdometryPosition(robot.drivetrain.follower.getPose());
    }
}
