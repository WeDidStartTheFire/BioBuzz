package org.firstinspires.ftc.teamcode.teleops.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Intake_JCTest1;

@TeleOp(name="JCTest2")
public class TeleOp_JCTest2 extends OpMode {

    Intake_JCTest1 intake;

    @Override
    public void init() {
        intake = new Intake_JCTest1(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger_pressed) {
            //When intake button is pressed, intake motor is turned on.
            intake.inTake();
        } else {
            intake.idle();
        }
        if (gamepad1.left_trigger_pressed) {
            //When intake button is pressed, intake motor is turned on.
            intake.outTake();
        } else {
            intake.idle();
        }
    }
}