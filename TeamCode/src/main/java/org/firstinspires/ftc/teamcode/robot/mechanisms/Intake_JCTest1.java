package org.firstinspires.ftc.teamcode.robot.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake_JCTest1{
    DcMotorEx intakeMotor;
    public Intake_JCTest1 (HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

    }
    public void inTake () {
        intakeMotor.setPower(.75);
    }
    public void outTake () {
        intakeMotor.setPower(-.75);
    }
    public void idle () {
        intakeMotor.setPower(0);
    }
}
