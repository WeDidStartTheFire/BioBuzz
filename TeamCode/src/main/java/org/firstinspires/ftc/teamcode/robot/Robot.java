package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MatchContext;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.mechanisms.ColorSensor;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.robot.mechanisms.LED;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Limelight;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Turret;

public class Robot {
    public Drivetrain drivetrain;
    public ColorSensor colorSensor;
    public Intake intake;
    public Limelight limelight;
    public Launcher launcher;
    public LED led;
    public Turret turret;
    public HardwareMap hardwareMap;
    private final MatchContext context;

    public Robot(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry,
                 MatchContext context) {
        this.context = context;
        this.hardwareMap = hardwareMap;
        TelemetryUtils tm = new TelemetryUtils(telemetry);
        drivetrain = new Drivetrain(hardwareMap, tm, this.context);
        intake = new Intake(hardwareMap, tm);
        colorSensor = new ColorSensor(hardwareMap, tm);
        led = new LED(hardwareMap, tm);
        limelight = new Limelight(hardwareMap, tm);
        launcher = new Launcher(hardwareMap, tm, drivetrain.poseGetter());
        turret = new Turret(hardwareMap, tm, drivetrain.poseGetter());
    }

    public void initBulkCache() {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    public void updateBulkCache() {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.clearBulkCache();
    }

    public MatchContext context() {
        return context;
    }
}
