package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.mechanisms.ColorSensor;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Feeder;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Indexer;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.robot.mechanisms.LED;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Limelight;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Turret;

import java.util.ArrayList;
import java.util.List;

public class Robot implements Subsystem {
    public Drivetrain drivetrain;
    public Feeder feeder;
    public ColorSensor colorSensor;
    public Intake intake;
    public Indexer indexer;
    public Limelight limelight;
    public Launcher launcher;
    public LED led;
    public Turret turret;
    public HardwareMap hardwareMap;
    private final List<Subsystem> subsystems = new ArrayList<>();

    public Robot(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry, boolean useOdometry) {
        this.hardwareMap = hardwareMap;
        TelemetryUtils tm = new TelemetryUtils(telemetry);
        intake = new Intake(hardwareMap, tm);
        drivetrain = new Drivetrain(hardwareMap, tm, useOdometry);
        led = new LED(hardwareMap, tm);
        feeder = new Feeder(hardwareMap, tm);
        colorSensor = new ColorSensor(hardwareMap, tm);
        indexer = new Indexer(hardwareMap, tm, colorSensor, feeder);
        limelight = new Limelight(hardwareMap, tm);
        launcher = new Launcher(hardwareMap, tm);
        turret = new Turret(hardwareMap, tm);

        subsystems.add(drivetrain);
        subsystems.add(intake);
        subsystems.add(led);
        subsystems.add(feeder);
        subsystems.add(indexer);
        subsystems.add(colorSensor);
        subsystems.add(limelight);
        subsystems.add(launcher);
        subsystems.add(turret);
    }

    public void initBulkCache() {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    public void updateBulkCache() {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.clearBulkCache();
    }

    @Override
    public void update() {
        for (Subsystem subsystem : subsystems) subsystem.update();
    }

    @Override
    public void stop() {
        for (Subsystem subsystem : subsystems) subsystem.stop();
    }

}