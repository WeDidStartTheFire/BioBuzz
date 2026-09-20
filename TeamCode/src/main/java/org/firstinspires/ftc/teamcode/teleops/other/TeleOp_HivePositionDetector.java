package org.firstinspires.ftc.teamcode.teleops.other;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Limelight;

@TeleOp(name = "TeleOp_HivePositionDetector", group = "C")
public class TeleOp_HivePositionDetector extends OpMode {
    TelemetryUtils tm;
    Limelight limelight;

    @Override
    public void init() {
        tm = new TelemetryUtils(telemetry);
        limelight = new Limelight(hardwareMap, tm);
        limelight.start();
        tm.update();
    }

    @Override
    public void loop() {
        tm.print(limelight.getHivePosition());
        tm.update();
    }
}
