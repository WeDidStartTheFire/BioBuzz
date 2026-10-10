package org.firstinspires.ftc.teamcode.teleops.other;

import static org.firstinspires.ftc.teamcode.TelemetryUtils.PrintLevel.INFO;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.PrintLevel.VERBOSE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.enums.Hive;
import org.firstinspires.ftc.teamcode.enums.RobotEnvironment;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Limelight;

@TeleOp(name = "TeleOp_HivePositionDetector", group = "C")
public class TeleOp_HivePositionDetector extends OpMode {
    TelemetryUtils tm;
    Limelight limelight;

    @Override
    public void init() {
        tm = new TelemetryUtils(telemetry);
        tm.setEnvironment(RobotEnvironment.DEBUG);
        limelight = new Limelight(hardwareMap, tm);
        limelight.start();
        tm.update();
    }

    @Override
    public void loop() {
        tm.print("Blue", limelight.getHivePosition(Color.BLUE), INFO);
        tm.print("Red", limelight.getHivePosition(Color.RED), INFO);

        tm.print("Blue Audience Y", limelight.getAverageHiveY(Hive.BLUE_AUDIENCE), VERBOSE);
        tm.print("Blue Stage Y", limelight.getAverageHiveY(Hive.BLUE_STAGE), VERBOSE);
        tm.print("Red Audience Y", limelight.getAverageHiveY(Hive.RED_AUDIENCE), VERBOSE);
        tm.print("Red Stage Y", limelight.getAverageHiveY(Hive.RED_STAGE), VERBOSE);

        tm.updateOnlyPanels();
    }
}
