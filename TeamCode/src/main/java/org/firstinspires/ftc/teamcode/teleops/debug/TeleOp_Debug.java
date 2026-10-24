package org.firstinspires.ftc.teamcode.teleops.debug;

import static org.firstinspires.ftc.teamcode.MatchContext.Mode.TELEOP;
import static org.firstinspires.ftc.teamcode.utils.TelemetryUtils.ErrorLevel.LOW;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MatchContext;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.enums.RobotEnvironment;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;

@TeleOp(name = "Test", group = "Test")
@Configurable
public class TeleOp_Debug extends OpMode {

    public static double SPEED_MULTIPLIER = 0.75;
    public static double BASE_TURN_SPEED = 2.5;
    public static double MOTOR_SPEED = 1;
    private boolean wasDownA = false;
    private boolean wasDownB = false;
    private Servo servoA, servoB, servoC, servoD;
    private DcMotorEx motorA, motorB;
    private Robot robot;
    private TelemetryUtils tm;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, new MatchContext(TELEOP, Color.BLUE));

        tm = robot.tm();
        tm.setEnvironment(RobotEnvironment.PRACTICE);
        servoA = HardwareInitializer.init(hardwareMap, Servo.class, "servoA");
        servoB = HardwareInitializer.init(hardwareMap, Servo.class, "servoB");
        servoC = HardwareInitializer.init(hardwareMap, Servo.class, "servoC");
        servoD = HardwareInitializer.init(hardwareMap, Servo.class, "servoD");
        if (servoA == null) tm.warn(LOW, "servoA disconnected.");
        if (servoB == null) tm.warn(LOW, "servoB disconnected.");
        if (servoC == null) tm.warn(LOW, "servoC disconnected.");
        if (servoD == null) tm.warn(LOW, "servoD disconnected.");

        motorA = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "intakeMotor");
        motorB = HardwareInitializer.init(hardwareMap, DcMotorEx.class, "launcherMotorA");
        if (motorA == null) tm.warn(LOW, "intakeMotor disconnected.");
        if (motorB == null) tm.warn(LOW, "launcherMotorA disconnected.");
    }

    @Override
    public void loop() {
        // Slows down movement for better handling the more the right trigger is held down
        double slowdownMultiplier = (1.0 - gamepad1.right_trigger) * 0.7 + 0.3;
        // f (gamepad1.left_stick_button || gamepad1.right_stick_button) {
        // slowdownMultiplier *= 0.5; }

        double axial = ((-gamepad1.left_stick_y * SPEED_MULTIPLIER) * slowdownMultiplier);
        double lateral = ((gamepad1.left_stick_x * SPEED_MULTIPLIER) * slowdownMultiplier);
        double yaw = ((gamepad1.right_stick_x * BASE_TURN_SPEED) * slowdownMultiplier);

        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send calculated power to wheels
        robot.drivetrain.setMotorPowers(leftBackPower, rightBackPower, leftFrontPower,
                rightFrontPower);

        if (servoA != null) {
            if (gamepad1.a && !wasDownA) {
                if (servoA.getPosition() > 0.95) {
                    servoA.setPosition(0);
                    tm.addLastActionTelemetry("Set servoA to 0");
                } else {
                    servoA.setPosition(1);
                    tm.addLastActionTelemetry("Set servoA to 1");
                }
            }
            wasDownA = gamepad1.a;
        } else {
            tm.addLastActionTelemetry("servoA disconnected");
        }

        if (servoB != null) {
            if (gamepad1.b && !wasDownB) {
                if (servoB.getPosition() > 0.95) {
                    servoB.setPosition(0);
                    tm.addLastActionTelemetry("Set servoB to 0");
                } else {
                    servoB.setPosition(1);
                    tm.addLastActionTelemetry("Set servoB to 1");
                }
            }
            wasDownB = gamepad1.b;
        } else {
            tm.addLastActionTelemetry("servoB diconnected");
        }

        if (servoC != null) {
            if (gamepad1.x) {
                servoC.setPosition(1);
                tm.addLastActionTelemetry("Set servoC to 1");
            } else if (gamepad1.y) {
                servoC.setPosition(0);
                tm.addLastActionTelemetry("Set servoC to 0");
            } else {
                servoC.setPosition(0.5);
                tm.addLastActionTelemetry("Set servoC to 0.5");
            }
        } else {
            tm.addLastActionTelemetry("servoC disconnected");
        }

        if (servoD != null) {
            if (gamepad1.left_bumper) {
                servoD.setPosition(0);
                tm.addLastActionTelemetry("Set servoD to 0");
            } else if (gamepad1.right_bumper) {
                servoD.setPosition(1);
                tm.addLastActionTelemetry("Set servoD to 1");
            } else {
                servoD.setPosition(0.5);
                tm.addLastActionTelemetry("Set servoD to 0.5");
            }
        } else {
            tm.addLastActionTelemetry("servoD disconnected");
        }

        if (motorA != null) {
            if (gamepad1.dpad_up) {
                motorA.setPower(1 * MOTOR_SPEED);
            } else if (gamepad1.dpad_down) {
                motorA.setPower(-1 * MOTOR_SPEED);
            } else {
                motorA.setPower(0);
            }
        } else {
            tm.addLastActionTelemetry("motorA disconnected");
        }

        if (motorB != null) {
            if (gamepad1.dpad_right) {
                motorB.setPower(1 * MOTOR_SPEED);
            } else if (gamepad1.dpad_left) {
                motorB.setPower(-1 * MOTOR_SPEED);
            } else {
                motorB.setPower(0);
            }
        } else {
            tm.addLastActionTelemetry("motorB disconnected");
        }

        telemetry.update();
    }
}
