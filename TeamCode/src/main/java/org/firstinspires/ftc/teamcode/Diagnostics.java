package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.COLOR_SENSOR_A;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.COLOR_SENSOR_B;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.DRIVETRAIN_LEFT_BACK_MOTOR;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.DRIVETRAIN_LEFT_FRONT_MOTOR;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.DRIVETRAIN_RIGHT_BACK_MOTOR;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.DRIVETRAIN_RIGHT_FRONT_MOTOR;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.FEEDER_SERVO_A;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.FEEDER_SERVO_B;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.INDEXER_SERVO;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.INTAKE_MOTOR;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.INTAKE_SERVO_A;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.INTAKE_SERVO_B;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.INTAKE_SERVO_C;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.LAUNCHER_MOTOR_A;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.LAUNCHER_MOTOR_B;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.LED;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.LIMELIGHT;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.SPARKFUN_OTOS;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.TURRET_MOTOR;
import static org.firstinspires.ftc.teamcode.RobotConstants.Hardware.TURRET_TOUCH_SENSOR;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Feeder;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Indexer;

@Utility(name = "Diagnostics")
public class Diagnostics extends LinearOpMode {
    public DcMotorEx lf, lb, rf, rb, intakeMotor, launcherMotorA, launcherMotorB, turretMotor;
    public Servo feederServoA, feederServoB, indexerServo, led;
    public CRServo intakeServoA, intakeServoB, intakeServoC;
    public Limelight3A limelight;
    public ColorSensor colorSensor, colorSensorB;
    public TouchSensor turretTouchSensor;
    public SparkFunOTOS otos;
    public Indexer indexer;
    
    public TelemetryUtils tm;
    
    public void runOpMode() {
        tm = new TelemetryUtils(telemetry);
        lf = HardwareInitializer.init(hardwareMap, tm, DRIVETRAIN_LEFT_FRONT_MOTOR);
        lb = HardwareInitializer.init(hardwareMap, tm, DRIVETRAIN_LEFT_BACK_MOTOR);
        rf = HardwareInitializer.init(hardwareMap, tm, DRIVETRAIN_RIGHT_FRONT_MOTOR);
        rb = HardwareInitializer.init(hardwareMap, tm, DRIVETRAIN_RIGHT_BACK_MOTOR);

        feederServoA = HardwareInitializer.init(hardwareMap, tm, FEEDER_SERVO_A);
        feederServoB = HardwareInitializer.init(hardwareMap, tm, FEEDER_SERVO_B);

        indexerServo = HardwareInitializer.init(hardwareMap, tm, INDEXER_SERVO);

        intakeMotor = HardwareInitializer.init(hardwareMap, tm, INTAKE_MOTOR);
        intakeServoA = HardwareInitializer.init(hardwareMap, tm, INTAKE_SERVO_A);
        intakeServoB = HardwareInitializer.init(hardwareMap, tm, INTAKE_SERVO_B);
        intakeServoC = HardwareInitializer.init(hardwareMap, tm, INTAKE_SERVO_C);

        launcherMotorA = HardwareInitializer.init(hardwareMap, tm, LAUNCHER_MOTOR_A);
        launcherMotorB = HardwareInitializer.init(hardwareMap, tm, LAUNCHER_MOTOR_B);

        led = HardwareInitializer.init(hardwareMap, tm, LED);

        limelight = HardwareInitializer.init(hardwareMap, tm, LIMELIGHT);

        colorSensor = HardwareInitializer.init(hardwareMap, tm, COLOR_SENSOR_A);
        colorSensorB = HardwareInitializer.init(hardwareMap, tm, COLOR_SENSOR_B);

        turretMotor = HardwareInitializer.init(hardwareMap, tm, TURRET_MOTOR);
        turretTouchSensor = HardwareInitializer.init(hardwareMap, tm, TURRET_TOUCH_SENSOR);

        otos = HardwareInitializer.init(hardwareMap, tm, SPARKFUN_OTOS);

        indexer = new Indexer(hardwareMap, tm,
            new org.firstinspires.ftc.teamcode.robot.mechanisms.ColorSensor(hardwareMap, tm),
            new Feeder(hardwareMap, tm));

        tm.update();
        waitForStart();
        if (!opModeIsActive()) return;

        if (feederServoA != null && feederServoB != null) {
            feederServoA.setDirection(Servo.Direction.REVERSE);
            feederServoB.setPosition(.85);
            feederServoA.setPosition(.85);
            actionTm("Raising feeder servo", "Lowering feeder servo");
            sleep(1000);
            feederServoA.setPosition(0);
            feederServoB.setPosition(0);
            actionTm("Lowering feeder servo", "Indexer to second position");
            sleep(1000);
        }

        if (indexerServo != null){
            indexer.setPos(0.5);
            actionTm("Indexer to second position", "Indexer to third position");
            sleep(1000);
            indexer.setPos(1);
            actionTm("Indexer to third position", "Indexer to first position");
            sleep(1000);
            indexer.setPos(0);
            actionTm("Indexer to first position", "Intake moving");
            sleep(1000);
        }

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
        while(getRuntime() < 5.0){
            tm.print("Launcher Motor A Velocity",  launcherMotorA.getVelocity());
            tm.print("Launcher Motor B Velocity",  launcherMotorB.getVelocity());
            tm.update();
        }

        if (launcherMotorA != null) launcherMotorA.setVelocity(0);
        if (launcherMotorB != null) launcherMotorB.setVelocity(0);
    }

    private void actionTm(String current, String next){
        tm.print("Currently", current);
        tm.print("Next", next);
        tm.update();
    }
}
