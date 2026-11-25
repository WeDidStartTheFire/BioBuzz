package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.teleops.TeleOp_Lift;

public class Lift {
    private @Nullable DcMotor liftMotor;
    private TelemetryUtils tm;

    public Lift(HardwareMap hardwareMap, TelemetryUtils tm) {
        this.tm = tm;

        liftMotor = HardwareInitializer.init(hardwareMap, DcMotor.class, "lift");
        if (liftMotor == null) {tm.warn(TelemetryUtils.ErrorLevel.CRITICAL, "Lift motor is null!");}
    }

    public void setLiftMotorPower(float power) {
        if (liftMotor == null) {return;}
        liftMotor.setPower(power);
    }
}
