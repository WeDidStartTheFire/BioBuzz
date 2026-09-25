package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.INTAKE_MOTOR;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.INTAKE_SERVO_A;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.INTAKE_SERVO_B;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.INTAKE_SERVO_C;
import static java.lang.Math.abs;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;

public class Intake {

    private final @Nullable CRServo intakeServoA, intakeServoB, intakeServoC;
    private final @Nullable DcMotorEx intakeMotor;
    private double outsidePower = 0;
    private double insidePower = 0;

    public Intake(HardwareMap hardwareMap, TelemetryUtils tm) {
        intakeMotor = HardwareInitializer.init(hardwareMap, tm, INTAKE_MOTOR);

        intakeServoA = HardwareInitializer.init(hardwareMap, tm, INTAKE_SERVO_A);
        intakeServoB = HardwareInitializer.init(hardwareMap, tm, INTAKE_SERVO_B);
        intakeServoC = HardwareInitializer.init(hardwareMap, tm, INTAKE_SERVO_C);
        if (intakeServoC != null) intakeServoC.setDirection(REVERSE);
    }

    /**
     * Powers all intake servos and motor
     *
     * @param power Power on [-1, 1]. Negative is inward, positive is outward.
     */
    public void power(double power) {
        powerInside(power);
        powerOutside(power);
    }

    /**
     * Powers the inside intake servos
     *
     * @param power Power on [-1, 1]. Negative is inward, positive is outward.
     */
    public void powerInside(double power) {
        if (abs(insidePower - power) < .02) return;
        insidePower = power;
        if (intakeServoA != null) intakeServoA.setPower(power);
        if (intakeServoC != null) intakeServoC.setPower(power);
    }

    /**
     * Powers the outside intake motor and roller servo
     *
     * @param power Power on [-1, 1]. Negative is inward, positive is outward.
     */
    public void powerOutside(double power) {
        if (abs(outsidePower - power) < .02) return;
        outsidePower = power;
        if (intakeMotor != null) intakeMotor.setPower(power);
        if (intakeServoB != null) intakeServoB.setPower(power);
    }
}
