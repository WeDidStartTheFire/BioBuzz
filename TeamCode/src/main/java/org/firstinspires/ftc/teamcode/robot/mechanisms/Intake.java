package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.enums.Hardware.INTAKE_MOTOR;
import static org.firstinspires.ftc.teamcode.enums.Hardware.INTAKE_SERVO_A;
import static org.firstinspires.ftc.teamcode.enums.Hardware.INTAKE_SERVO_B;
import static org.firstinspires.ftc.teamcode.enums.Hardware.INTAKE_SERVO_C;
import static java.lang.Math.abs;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.BlazeMotor;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;

public class Intake {

    private final @Nullable CRServo intakeServoA, intakeServoB, intakeServoC;
    private final BlazeMotor intakeMotor;
    private double cachedOutsidePower = 0;
    private double cachedInsidePower = 0;
    private double outsidePower = 0;
    private double insidePower = 0;

    public Intake(HardwareMap hardwareMap, TelemetryUtils tm) {
        intakeMotor = new BlazeMotor(hardwareMap, tm, INTAKE_MOTOR);

        intakeServoA = HardwareInitializer.init(hardwareMap, tm, INTAKE_SERVO_A);
        intakeServoB = HardwareInitializer.init(hardwareMap, tm, INTAKE_SERVO_B);
        intakeServoC = HardwareInitializer.init(hardwareMap, tm, INTAKE_SERVO_C);
        if (intakeServoC != null) intakeServoC.setDirection(REVERSE);
    }

    public void calculate() {
        if (abs(insidePower - cachedInsidePower) > .02) {
            cachedInsidePower = insidePower;
            if (intakeServoA != null) intakeServoA.setPower(insidePower);
            if (intakeServoC != null) intakeServoC.setPower(insidePower);
        }
        if (abs(outsidePower - cachedOutsidePower) > .02) {
            cachedOutsidePower = outsidePower;
            intakeMotor.setPower(outsidePower);
            if (intakeServoB != null) intakeServoB.setPower(outsidePower);
        }
    }

    public void stop() {
        intakeMotor.writeZeroPower();
        if (intakeServoA != null) intakeServoA.setPower(0);
        if (intakeServoB != null) intakeServoB.setPower(0);
        if (intakeServoC != null) intakeServoC.setPower(0);
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
        insidePower = power;
    }

    /**
     * Powers the outside intake motor and roller servo
     *
     * @param power Power on [-1, 1]. Negative is inward, positive is outward.
     */
    public void powerOutside(double power) {
        outsidePower = power;
    }
}
