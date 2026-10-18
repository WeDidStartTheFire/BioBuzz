package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.enums.Hardware.INTAKE_MOTOR;
import static org.firstinspires.ftc.teamcode.enums.Hardware.INTAKE_SERVO_A;
import static org.firstinspires.ftc.teamcode.enums.Hardware.INTAKE_SERVO_B;
import static org.firstinspires.ftc.teamcode.enums.Hardware.INTAKE_SERVO_C;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

public class Intake implements Subsystem {
    private final @Nullable CRServo intakeServoA, intakeServoB, intakeServoC;
    private final @Nullable DcMotorEx intakeMotor;
    private double outsidePower = 0;
    private double insidePower = 0;

    public enum Behavior {
        OFF, INTAKE, OUTTAKE
    }

    private Behavior behavior = Behavior.OFF;

    public Intake(HardwareMap hardwareMap, TelemetryUtils tm) {
        intakeMotor = HardwareInitializer.init(hardwareMap, tm, INTAKE_MOTOR);

        intakeServoA = HardwareInitializer.init(hardwareMap, tm, INTAKE_SERVO_A);
        intakeServoB = HardwareInitializer.init(hardwareMap, tm, INTAKE_SERVO_B);
        intakeServoC = HardwareInitializer.init(hardwareMap, tm, INTAKE_SERVO_C);
        if (intakeServoC != null) intakeServoC.setDirection(REVERSE);
    }

    public void setBehavior(Behavior behavior) {
        this.behavior = behavior;
    }

    @Override
    public void update() {
        switch (behavior) {
            case OFF:
                insidePower = 0;
                outsidePower = 0;
                break;

            case INTAKE:
                insidePower = -1;
                outsidePower = -1;
                break;

            case OUTTAKE:
                insidePower = 1;
                outsidePower = 1;
                break;
        }
        if (intakeMotor != null) intakeMotor.setPower(outsidePower);
        if (intakeServoB != null) intakeServoB.setPower(outsidePower);
        if (intakeServoA != null) intakeServoA.setPower(insidePower);
        if (intakeServoC != null) intakeServoC.setPower(insidePower);
    }

    @Override
    public void stop() {
        behavior = Behavior.OFF;
        if (intakeMotor != null) intakeMotor.setPower(0);
        if (intakeServoB != null) intakeServoB.setPower(0);
        if (intakeServoA != null) intakeServoA.setPower(0);
        if (intakeServoC != null) intakeServoC.setPower(0);
    }
}
