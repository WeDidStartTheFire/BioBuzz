package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.mechanisms.ColorSensor;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.mechanisms.DrivetrainRefactor;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Feeder;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Indexer;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.robot.mechanisms.IntakeRefactor;
import org.firstinspires.ftc.teamcode.robot.mechanisms.LED;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Limelight;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Turret;

import java.util.ArrayList;
import java.util.List;

public class RobotRefactor implements Subsystem {
    public DrivetrainRefactor drivetrain;
    public Feeder feeder;
    public ColorSensor colorSensor;
    public IntakeRefactor intake;
    public Indexer indexer;
    public Limelight limelight;
    public Launcher launcher;
    public LED led;
    public Turret turret;
    public HardwareMap hardwareMap;
    private final List<Subsystem> subsystems = new ArrayList<>();

    public RobotRefactor(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry, boolean useOdometry) {
        this.hardwareMap = hardwareMap;
        TelemetryUtils tm = new TelemetryUtils(telemetry);
        intake = new IntakeRefactor(hardwareMap, tm);
        drivetrain = new DrivetrainRefactor(hardwareMap, tm, true);
        led = new LED(hardwareMap, tm);
        feeder = new Feeder(hardwareMap, tm);
        colorSensor = new ColorSensor(hardwareMap, tm);
        indexer = new Indexer(hardwareMap, tm, colorSensor, feeder);

        subsystems.add(drivetrain);
        subsystems.add(intake);
        subsystems.add(led);
        subsystems.add(feeder);
        subsystems.add(indexer);
        subsystems.add(colorSensor);
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