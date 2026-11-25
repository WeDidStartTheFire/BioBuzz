package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TelemetryUtils;

public class Claw {
    private final @Nullable Servo claw;
    private final TelemetryUtils tm;
    private boolean isClawOpened;

    public Claw(HardwareMap hardwareMap, TelemetryUtils tm) {
        this.tm = tm;

        claw = HardwareInitializer.init(hardwareMap, Servo.class, "claw");
        if (claw == null) {
            tm.warn(TelemetryUtils.ErrorLevel.CRITICAL, "Claw servo is null!");
        }
    }

    public void setClawPosition(float position) {
        if (claw == null) return;
        claw.setPosition(position);
    }

    public void toggleClaw() {
        if (claw == null) return;
        if (!isClawOpened) {
            setClawPosition(1);
            isClawOpened = true;
        } else {
            setClawPosition(0);
            isClawOpened = false;
        }
        tm.print("Is claw openned?", isClawOpened);
    }
}
