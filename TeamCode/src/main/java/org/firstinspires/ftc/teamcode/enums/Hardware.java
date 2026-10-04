package org.firstinspires.ftc.teamcode.enums;

import static org.firstinspires.ftc.teamcode.TelemetryUtils.ErrorLevel.CRITICAL;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.ErrorLevel.HIGH;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.ErrorLevel.LOW;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.ErrorLevel.MEDIUM;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.TelemetryUtils;

public enum Hardware {
    DRIVETRAIN_LEFT_FRONT_MOTOR("Left Front Drivetrain Motor", DcMotorEx.class, "leftFront", Hub.CONTROL, PortType.MOTOR, 1, CRITICAL),
    DRIVETRAIN_LEFT_BACK_MOTOR("Left Back Drivetrain Motor", DcMotorEx.class, "leftBack", Hub.CONTROL, PortType.MOTOR, 3, CRITICAL),
    DRIVETRAIN_RIGHT_FRONT_MOTOR("Right Front Drivetrain Motor", DcMotorEx.class, "rightFront", Hub.CONTROL, PortType.MOTOR, 0, CRITICAL),
    DRIVETRAIN_RIGHT_BACK_MOTOR("Right Back Drivetrain Motor", DcMotorEx.class, "rightBack", Hub.CONTROL, PortType.MOTOR, 2, CRITICAL),
    FEEDER_SERVO_A("Feeder Servo A", Servo.class, "feederServoA", Hub.EXPANSION, PortType.SERVO, 0, HIGH),
    FEEDER_SERVO_B("Feeder Servo B", Servo.class, "feederServoB", Hub.EXPANSION, PortType.SERVO, 1, HIGH),
    COLOR_SENSOR_A("Color Sensor A", RevColorSensorV3.class, "colorSensorA", Hub.CONTROL, PortType.I2C, 2, CRITICAL),
    COLOR_SENSOR_B("Color Sensor B", RevColorSensorV3.class, "colorSensorB", Hub.CONTROL, PortType.I2C, 3, CRITICAL),
    LED("LED", Servo.class, "led", Hub.CONTROL, PortType.SERVO, 0, CRITICAL),
    INDEXER_SERVO("Indexer Servo", Servo.class, "indexerServo", Hub.EXPANSION, PortType.SERVO, 2, CRITICAL),
    TURRET_MOTOR("Turret Motor", DcMotorEx.class, "turretMotor", Hub.EXPANSION, PortType.MOTOR, 3, HIGH),
    TOUCH_SENSOR_A("Touch Sensor A", TouchSensor.class, "touchSensorA", Hub.UNKNOWN, PortType.DIGITAL, -1, LOW),
    TOUCH_SENSOR_B("Touch Sensor B", TouchSensor.class, "touchSensorB", Hub.UNKNOWN, PortType.DIGITAL, -1, LOW),
    TURRET_TOUCH_SENSOR("Turret Touch Sensor", TouchSensor.class, "turretTouchSensor", Hub.EXPANSION, PortType.DIGITAL, 2, HIGH),
    LAUNCHER_MOTOR_A("Launcher Motor A", DcMotorEx.class, "launcherMotorA", Hub.EXPANSION, PortType.MOTOR, 1, CRITICAL),
    LAUNCHER_MOTOR_B("Launcher Motor B", DcMotorEx.class, "launcherMotorB", Hub.EXPANSION, PortType.MOTOR, 2, CRITICAL),
    LIMELIGHT("Limelight", Limelight3A.class, "limelight", Hub.CONTROL, PortType.USB, -1, MEDIUM),
    SPARKFUN_OTOS("SparkFun OTOS", SparkFunOTOS.class, "sensorOtos", Hub.CONTROL, PortType.I2C, 1, CRITICAL),
    INTAKE_MOTOR("Intake Motor", DcMotorEx.class, "intakeMotor", Hub.EXPANSION, PortType.MOTOR, 0, HIGH),
    INTAKE_SERVO_A("Intake Servo A", CRServo.class, "intakeServoA", Hub.EXPANSION, PortType.SERVO, 3, HIGH),
    INTAKE_SERVO_B("Intake Servo B (roller)", CRServo.class, "intakeServoB", Hub.EXPANSION, PortType.SERVO, 4, LOW),
    INTAKE_SERVO_C("Intake Servo C", CRServo.class, "intakeServoC", Hub.EXPANSION, PortType.SERVO, 5, HIGH),
    IMU("IMU", IMU.class, "imu", Hub.CONTROL, PortType.I2C, 0, MEDIUM);

    public final String name;
    public final Class<?> classOrInterface;
    public final String deviceName;
    public final Hub hub;
    public final PortType portType;
    public final int port;
    public final TelemetryUtils.ErrorLevel errorLevel;

    Hardware(String name, Class<?> classOrInterface, String deviceName, Hub hub, PortType portType, int port, TelemetryUtils.ErrorLevel errorLevel) {
        this.name = name;
        this.classOrInterface = classOrInterface;
        this.deviceName = deviceName;
        this.hub = hub;
        this.portType = portType;
        this.port = port;
        this.errorLevel = errorLevel;
    }

    public String getWarnMessage() {
        return name + " is disconnected. Please check " + hub.toString() + " " + portType.toString() + " " + port;
    }
}
