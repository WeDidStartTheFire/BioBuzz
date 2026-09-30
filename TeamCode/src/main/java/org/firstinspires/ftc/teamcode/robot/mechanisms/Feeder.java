package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static org.firstinspires.ftc.teamcode.TelemetryUtils.ErrorLevel.HIGH;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.ErrorLevel.LOW;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

public class Feeder implements Subsystem {
    private final @Nullable Servo feederServoA, feederServoB;
    private final @Nullable TouchSensor touchSensorA, touchSensorB;
    private double feederPos = -1;
    private double feederPosTarget;

    public Feeder(HardwareMap hardwareMap, TelemetryUtils tm) {
        feederServoA = HardwareInitializer.init(hardwareMap, Servo.class, "feederServoA");
        feederServoB = HardwareInitializer.init(hardwareMap, Servo.class, "feederServoB");
        if (feederServoA != null) feederServoA.setDirection(Servo.Direction.REVERSE);
        if (feederServoA == null && feederServoB == null)
            tm.warn(HIGH, "Both feeder servos are disconnected. Check Expansion Hub" +
                    " servo ports 0 and 1.");
        else if (feederServoA == null || feederServoB == null)
            tm.warn(HIGH, "One feeder servo is disconnected. Check Expansion Hub" +
                    " servo ports 0 and 1.");

        touchSensorA = HardwareInitializer.init(hardwareMap, TouchSensor.class, "touchSensorA");
        touchSensorB = HardwareInitializer.init(hardwareMap, TouchSensor.class, "touchSensorB");
        if (touchSensorA == null)
            tm.warn(LOW, "Touch Sensor A for the feeder is disconnected.");
        if (touchSensorB == null)
            tm.warn(LOW, "Touch Sensor B for the feeder is disconnected.");
    }

    /**
     * Returns true if the feeder is up. Based on the touch sensors.
     *
     * @return true if the feeder is up
     */
    public boolean isUp() {
        return (touchSensorA != null && !touchSensorA.isPressed()) &&
                (touchSensorB != null && !touchSensorB.isPressed());
    }

    /**
     * Returns if the feeder is up/moving up. Based on the feeder's set position.
     *
     * @return Whether the feeder is up/moving up
     */
    public boolean isGoalUp() {
        return getGoalPos() >= 0.2;
    }

    /**
     * Returns the goal position of the feeder. -1 if not yet set.
     *
     * @return the goal position of the feeder
     */
    public double getGoalPos() {
        return feederPos;
    }

    /**
     * Raises the feeder.
     */
    public void raise() {
        if (feederServoA == null || feederServoB == null || feederPos == 1) return;
        feederPos = 1;
        feederPosTarget = .85;
    }

    /**
     * Retracts the feeder.
     */
    public void retract() {
        if (feederServoA == null || feederServoB == null || feederPos == 0) return;
        feederPos = 0;
        feederPosTarget = 0.05;
    }

    @Override
    public void update() {
        if (feederServoA != null) feederServoA.setPosition(feederPosTarget);
        if (feederServoB != null) feederServoB.setPosition(feederPosTarget);
    }

    @Override
    public void stop() {
        if (feederServoA != null) feederServoA.setPosition(0);
        if (feederServoB != null) feederServoB.setPosition(0);
    }
}
