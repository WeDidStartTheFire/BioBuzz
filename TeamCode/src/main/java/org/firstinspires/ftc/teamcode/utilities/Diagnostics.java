package org.firstinspires.ftc.teamcode.utilities;

import static org.firstinspires.ftc.teamcode.TelemetryUtils.PrintLevel.VERBOSE;
import static org.firstinspires.ftc.teamcode.enums.Hardware.COLOR_SENSOR_A;
import static org.firstinspires.ftc.teamcode.enums.Hardware.COLOR_SENSOR_B;
import static org.firstinspires.ftc.teamcode.enums.Hardware.DRIVETRAIN_LEFT_BACK_MOTOR;
import static org.firstinspires.ftc.teamcode.enums.Hardware.DRIVETRAIN_LEFT_FRONT_MOTOR;
import static org.firstinspires.ftc.teamcode.enums.Hardware.DRIVETRAIN_RIGHT_BACK_MOTOR;
import static org.firstinspires.ftc.teamcode.enums.Hardware.DRIVETRAIN_RIGHT_FRONT_MOTOR;
import static org.firstinspires.ftc.teamcode.enums.Hardware.FEEDER_SERVO_A;
import static org.firstinspires.ftc.teamcode.enums.Hardware.FEEDER_SERVO_B;
import static org.firstinspires.ftc.teamcode.enums.Hardware.IMU;
import static org.firstinspires.ftc.teamcode.enums.Hardware.INDEXER_SERVO;
import static org.firstinspires.ftc.teamcode.enums.Hardware.INTAKE_MOTOR;
import static org.firstinspires.ftc.teamcode.enums.Hardware.INTAKE_SERVO_A;
import static org.firstinspires.ftc.teamcode.enums.Hardware.INTAKE_SERVO_B;
import static org.firstinspires.ftc.teamcode.enums.Hardware.INTAKE_SERVO_C;
import static org.firstinspires.ftc.teamcode.enums.Hardware.LAUNCHER_MOTOR_A;
import static org.firstinspires.ftc.teamcode.enums.Hardware.LAUNCHER_MOTOR_B;
import static org.firstinspires.ftc.teamcode.enums.Hardware.LED;
import static org.firstinspires.ftc.teamcode.enums.Hardware.LIMELIGHT;
import static org.firstinspires.ftc.teamcode.enums.Hardware.SPARKFUN_OTOS;
import static org.firstinspires.ftc.teamcode.enums.Hardware.TOUCH_SENSOR_A;
import static org.firstinspires.ftc.teamcode.enums.Hardware.TOUCH_SENSOR_B;
import static org.firstinspires.ftc.teamcode.enums.Hardware.TURRET_MOTOR;
import static org.firstinspires.ftc.teamcode.enums.Hardware.TURRET_TOUCH_SENSOR;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.enums.RobotEnvironment;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;

@Utility(name = "Diagnostics")
public class Diagnostics extends LinearOpMode {
    public @Nullable DcMotorEx intakeMotor, launcherMotorA, launcherMotorB;
    public @Nullable CRServo intakeServoA, intakeServoB, intakeServoC;

    public TelemetryUtils tm;

    public void runOpMode() {
        tm = new TelemetryUtils(telemetry);
        tm.setEnvironment(RobotEnvironment.PRACTICE);

        HardwareInitializer.init(hardwareMap, tm, IMU);

        HardwareInitializer.init(hardwareMap, tm, DRIVETRAIN_LEFT_FRONT_MOTOR);
        HardwareInitializer.init(hardwareMap, tm, DRIVETRAIN_LEFT_BACK_MOTOR);
        HardwareInitializer.init(hardwareMap, tm, DRIVETRAIN_RIGHT_FRONT_MOTOR);
        HardwareInitializer.init(hardwareMap, tm, DRIVETRAIN_RIGHT_BACK_MOTOR);

        intakeMotor = HardwareInitializer.init(hardwareMap, tm, INTAKE_MOTOR);
        intakeServoA = HardwareInitializer.init(hardwareMap, tm, INTAKE_SERVO_A);
        intakeServoB = HardwareInitializer.init(hardwareMap, tm, INTAKE_SERVO_B);
        intakeServoC = HardwareInitializer.init(hardwareMap, tm, INTAKE_SERVO_C);

        launcherMotorA = HardwareInitializer.init(hardwareMap, tm, LAUNCHER_MOTOR_A);
        launcherMotorB = HardwareInitializer.init(hardwareMap, tm, LAUNCHER_MOTOR_B);

        HardwareInitializer.init(hardwareMap, tm, LED);

        HardwareInitializer.init(hardwareMap, tm, TURRET_MOTOR);
        HardwareInitializer.init(hardwareMap, tm, TURRET_TOUCH_SENSOR);

        HardwareInitializer.init(hardwareMap, tm, LIMELIGHT);
        HardwareInitializer.init(hardwareMap, tm, SPARKFUN_OTOS);

        HardwareInitializer.init(hardwareMap, tm, FEEDER_SERVO_A);
        HardwareInitializer.init(hardwareMap, tm, FEEDER_SERVO_B);
        HardwareInitializer.init(hardwareMap, tm, TOUCH_SENSOR_A);
        HardwareInitializer.init(hardwareMap, tm, TOUCH_SENSOR_B);

        HardwareInitializer.init(hardwareMap, tm, INDEXER_SERVO);
        HardwareInitializer.init(hardwareMap, tm, COLOR_SENSOR_A);
        HardwareInitializer.init(hardwareMap, tm, COLOR_SENSOR_B);

        tm.update();
        waitForStart();
        if (!opModeIsActive()) return;

        if (intakeServoA != null) intakeServoA.setPower(1);
        if (intakeServoC != null) intakeServoC.setPower(1);
        if (intakeMotor != null) intakeMotor.setPower(1);
        if (intakeServoB != null) intakeServoB.setPower(1);
        actionTm("Intake moving", "Spinning launcher motors");
        sleep(1000);
        if (intakeServoA != null) intakeServoA.setPower(0);
        if (intakeServoC != null) intakeServoC.setPower(0);
        if (intakeMotor != null) intakeMotor.setPower(0);
        if (intakeServoB != null) intakeServoB.setPower(0);

        if (launcherMotorA != null) launcherMotorA.setVelocity(1000);
        if (launcherMotorB != null) launcherMotorB.setVelocity(1000);
        actionTm("Spinning launcher motors", "End");
        resetRuntime();
        while (getRuntime() < 5.0 && launcherMotorA != null && launcherMotorB != null) {
            tm.print("Launcher Motor A Velocity", launcherMotorA.getVelocity(), VERBOSE);
            tm.print("Launcher Motor B Velocity", launcherMotorB.getVelocity(), VERBOSE);
            tm.update();
        }

        if (launcherMotorA != null) launcherMotorA.setVelocity(0);
        if (launcherMotorB != null) launcherMotorB.setVelocity(0);
    }

    private void actionTm(String current, String next) {
        tm.print("Currently", current, VERBOSE);
        tm.print("Next", next, VERBOSE);
        tm.update();
    }
}
