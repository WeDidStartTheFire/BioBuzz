package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TelemetryUtils;

public class Lift {
    private final @Nullable DcMotor liftMotor;
    private final TelemetryUtils tm;
    private double lastPower = 2;

    public Lift(HardwareMap hardwareMap, TelemetryUtils tm) {
        this.tm = tm;

        liftMotor = HardwareInitializer.init(hardwareMap, DcMotor.class, "lift");
        if (liftMotor == null) tm.warn(TelemetryUtils.ErrorLevel.CRITICAL, "Lift motor is null!");
        else liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setLiftMotorPower(double power) {
        if (liftMotor == null) return;
        power = Math.clamp(power, -1, 1);
        if (power == lastPower) return;
        lastPower = power;
        liftMotor.setPower(power);
    }
}
