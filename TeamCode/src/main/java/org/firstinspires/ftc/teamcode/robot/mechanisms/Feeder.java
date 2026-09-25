package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.FEEDER_SERVO_A;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.FEEDER_SERVO_B;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.TOUCH_SENSOR_A;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.TOUCH_SENSOR_B;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;

public class Feeder {
    private final @Nullable Servo feederServoA, feederServoB;
    private final @Nullable TouchSensor touchSensorA, touchSensorB;
    private double feederPos = -1;

    public Feeder(HardwareMap hardwareMap, TelemetryUtils tm) {
        feederServoA = HardwareInitializer.init(hardwareMap, tm, FEEDER_SERVO_A);
        feederServoB = HardwareInitializer.init(hardwareMap, tm, FEEDER_SERVO_B);
        if (feederServoA != null) feederServoA.setDirection(Servo.Direction.REVERSE);

        touchSensorA = HardwareInitializer.init(hardwareMap, tm, TOUCH_SENSOR_A);
        touchSensorB = HardwareInitializer.init(hardwareMap, tm, TOUCH_SENSOR_B);
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
        feederServoB.setPosition(.85);
        feederServoA.setPosition(.85);
    }

    /**
     * Retracts the feeder.
     */
    public void retract() {
        if (feederServoA == null || feederServoB == null || feederPos == 0) return;
        feederPos = 0;
        feederServoA.setPosition(0.05);
        feederServoB.setPosition(0.05);
    }
}
