package org.firstinspires.ftc.teamcode.utilities;

import static org.firstinspires.ftc.teamcode.MatchContext.Mode.TELEOP;
import static org.firstinspires.ftc.teamcode.utils.TelemetryUtils.PrintLevel.INFO;
import static org.firstinspires.ftc.teamcode.utils.TelemetryUtils.PrintLevel.VERBOSE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MatchContext;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.enums.Hive;
import org.firstinspires.ftc.teamcode.enums.RobotEnvironment;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Limelight;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;

@TeleOp(name = "Hive Position Detector")
public class HivePositionDetector extends OpMode {
    private TelemetryUtils tm;
    private Limelight limelight;
    Robot robot;

    @Override
    public void init() {
        tm = new TelemetryUtils(telemetry);
        tm.setEnvironment(RobotEnvironment.DEBUG);
        limelight = new Limelight(hardwareMap, tm);
        robot = new Robot(hardwareMap, telemetry, new MatchContext(TELEOP, Color.BLUE));
        robot.drivetrain.useKalmanFollower();
        limelight.start();
        tm.update();
    }

    @Override
    public void loop() {
        robot.drivetrain.update();
        tm.drawRobot(robot.drivetrain.follower);
        tm.print("Blue", limelight.getHivePosition(Color.BLUE), INFO);
        tm.print("Red", limelight.getHivePosition(Color.RED), INFO);

        tm.print("Blue Audience Y", limelight.getAverageHiveY(Hive.BLUE_AUDIENCE), VERBOSE);
        tm.print("Blue Stage Y", limelight.getAverageHiveY(Hive.BLUE_STAGE), VERBOSE);
        tm.print("Red Audience Y", limelight.getAverageHiveY(Hive.RED_AUDIENCE), VERBOSE);
        tm.print("Red Stage Y", limelight.getAverageHiveY(Hive.RED_STAGE), VERBOSE);

        tm.updateOnlyPanels();
    }
}
