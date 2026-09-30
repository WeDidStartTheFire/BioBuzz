package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.ErrorLevel.HIGH;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.ErrorLevel.LOW;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

public class IntakeRefactor implements Subsystem {
    private final @Nullable CRServo intakeServoA, intakeServoB, intakeServoC;
    private final @Nullable DcMotorEx intakeMotor;
    private double outsidePower = 0;
    private double insidePower = 0;
    public static enum Behavior {
        OFF, INTAKE, OUTTAKE
    }
    private Behavior behavior = Behavior.OFF;

    public IntakeRefactor(HardwareMap hardwareMap, TelemetryUtils tm) {
        intakeMotor = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "intakeMotor");
        if (intakeMotor == null)
            tm.warn(HIGH, "Intake Motor disconnected. Check Expansion hub motor port 0");

        intakeServoA = HardwareInitializer.init(hardwareMap, CRServo.class, "intakeServoA");
        intakeServoB = HardwareInitializer.init(hardwareMap, CRServo.class, "intakeServoB");
        intakeServoC = HardwareInitializer.init(hardwareMap, CRServo.class, "intakeServoC");
        if (intakeServoA == null)
            tm.warn(HIGH, "Intake Servo A disconnected. Check Expansion Hub servo port 3");
        if (intakeServoB == null)
            tm.warn(LOW, "Intake Servo B (roller) disconnected. Check Expansion Hub servo port 4");
        if (intakeServoC == null)
            tm.warn(HIGH, "Intake Servo C disconnected. Check Expansion Hub servo port 5");
        else intakeServoC.setDirection(REVERSE);
    }

    public void setBehavior(Behavior behavior) { this.behavior = behavior; }

    public Behavior getBehavior() { return behavior; }

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
