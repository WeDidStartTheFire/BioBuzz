package org.firstinspires.ftc.teamcode.robot.mechanisms;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;

public class Motor {
    private final @Nullable DcMotorEx motor;

    public Motor(HardwareMap hardwareMap, TelemetryUtils tm) {
        motor = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "Motor");
        if (motor == null) { tm.warn(TelemetryUtils.ErrorLevel.HIGH, "Motor is returning null! Please check if its plugged in."); }
    }

    /**
     * Makes the motor spin
     */
    public void spin() { if (motor != null) { motor.setPower(1); } }

    /**
     *  Makes the motor spin
     * @param power Changes the power of the motor (-1 to 1)
     */
    public void spin(double power) { if (motor != null) { motor.setPower(power); } }

    /**
     * Stops the motor
     */
    public void stopMotor() { if (motor != null) { motor.setPower(0); } }
}
