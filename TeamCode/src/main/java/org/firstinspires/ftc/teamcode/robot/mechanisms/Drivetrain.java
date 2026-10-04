package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.ErrorLevel.CRITICAL;
import static org.firstinspires.ftc.teamcode.constants.DrivetrainConstants.IMU_PARAMS;
import static org.firstinspires.ftc.teamcode.enums.Hardware.DRIVETRAIN_LEFT_BACK_MOTOR;
import static org.firstinspires.ftc.teamcode.enums.Hardware.DRIVETRAIN_LEFT_FRONT_MOTOR;
import static org.firstinspires.ftc.teamcode.enums.Hardware.DRIVETRAIN_RIGHT_BACK_MOTOR;
import static org.firstinspires.ftc.teamcode.enums.Hardware.DRIVETRAIN_RIGHT_FRONT_MOTOR;
import static org.firstinspires.ftc.teamcode.enums.Hardware.SPARKFUN_OTOS;

import androidx.annotation.Nullable;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MatchContext;
import org.firstinspires.ftc.teamcode.PoseGetter;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.enums.Hardware;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;

public class Drivetrain implements PoseGetter {
    private DcMotorEx lf, lb, rf, rb;
    private final @Nullable IMU imu;
    public Follower follower;
    public final @Nullable SparkFunOTOS otos;
    private @Nullable Pose pose;
    private @Nullable Velocity vel;
    private boolean validPose = false;

    public volatile boolean loop = false;

    public TelemetryUtils tm;
    private final HardwareMap hardwareMap;

    /**
     * Configures the drivetrain to use Kalman filter localization.
     */
    public void useKalmanFollower() {
        follower = Constants.createKalmanFollower(hardwareMap);
    }

    /**
     * Configures the drivetrain to use Limelight localization.
     */
    public void useLimelightFollower() {
        follower = Constants.createLimelightFollower(hardwareMap);
    }

    /**
     * Configures the drivetrain to use Limelight localization.
     */
    public void useRedundantFollower() {
        follower = Constants.createRedundantFollower(hardwareMap);
    }

    /**
     * Initializes the drivetrain with hardware components.
     *
     * @param hardwareMap HardwareMap containing motor and sensor configurations
     * @param tm          TelemetryUtils instance for debugging output
     */
    public Drivetrain(HardwareMap hardwareMap, TelemetryUtils tm, MatchContext context) {
        this.tm = tm;
        this.hardwareMap = hardwareMap;
        follower = Constants.create(hardwareMap);
        otos = HardwareInitializer.init(hardwareMap, tm, SPARKFUN_OTOS);

        imu = HardwareInitializer.init(hardwareMap, tm, Hardware.IMU);
        if (imu != null) {
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
            if (!imu.initialize(IMU_PARAMS))
                throw new RuntimeException("IMU initialization failed");
            imu.resetYaw();
        }
        // Drive train
        lf = HardwareInitializer.init(hardwareMap, tm, DRIVETRAIN_LEFT_FRONT_MOTOR);
        lb = HardwareInitializer.init(hardwareMap, tm, DRIVETRAIN_LEFT_BACK_MOTOR);
        rf = HardwareInitializer.init(hardwareMap, tm, DRIVETRAIN_RIGHT_FRONT_MOTOR);
        rb = HardwareInitializer.init(hardwareMap, tm, DRIVETRAIN_RIGHT_BACK_MOTOR);
        if (isMotorDisconnected()) {
            tm.warn(CRITICAL, "At least one drive train motor is not connected, so all will be disabled");
            lf = lb = rf = rb = null;
        }

        if (lf != null) {
            lf.setDirection(REVERSE);
            lb.setDirection(REVERSE);
            rf.setDirection(DcMotorEx.Direction.FORWARD);
            rb.setDirection(DcMotorEx.Direction.FORWARD);

            if (context.isAuto()) setZeroPowerBehavior(BRAKE);
            else setZeroPowerBehavior(FLOAT);

            lb.setTargetPosition(lb.getCurrentPosition());
            rb.setTargetPosition(rb.getCurrentPosition());
            lf.setTargetPosition(lf.getCurrentPosition());
            rf.setTargetPosition(rf.getCurrentPosition());
        }
    }


    public PoseGetter poseGetter() {
        return new PoseGetter() {
            PoseGetter poseGetter;

            PoseGetter init(PoseGetter poseGetter) {
                this.poseGetter = poseGetter;
                return this;
            }

            @Nullable
            @Override
            public Pose pose() {
                if (poseGetter == null) return null;
                return poseGetter.pose();
            }

            @Nullable
            @Override
            public Velocity vel() {
                if (poseGetter == null) return null;
                return poseGetter.vel();
            }
        }.init(this);
    }

    public @Nullable Pose pose() {
        return pose;
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
        validPose = pose != null;
        this.pose = pose;
    }

    public boolean isPoseValid() {
        return validPose;
    }

    public void update() {
        if (follower == null) return;
        follower.update();
        pose = follower.pose();
        vel = follower.velocity();
    }

    public @Nullable Velocity vel() {
        return vel;
    }

    public boolean isMotorDisconnected() {
        return lf == null || lb == null || rf == null || rb == null;
    }

    public double getYaw(AngleUnit angleUnit) {
        if (imu == null) return 0;
        return imu.getRobotOrientation(INTRINSIC, ZYX, angleUnit).firstAngle;
    }

    public void setOtosBusSpeed(LynxI2cDeviceSynch.BusSpeed busSpeed) {
        if (otos == null) return;
        ((LynxI2cDeviceSynch) otos.getDeviceClient()).setBusSpeed(busSpeed);
    }

    /**
     * Sets the mode of all drive train motors to the same mode.
     *
     * @param mode The mode to set the motors to.
     */
    private void setMotorModes(DcMotor.RunMode mode) {
        if (isMotorDisconnected()) return;
        lf.setMode(mode);
        lb.setMode(mode);
        rf.setMode(mode);
        rb.setMode(mode);
    }

    /**
     * Sets the same power for all drivetrain motors.
     *
     * @param power Power value to set for all motors on [-1, 1]
     */
    public void setMotorPowers(double power) {
        setMotorPowers(power, power, power, power);
    }

    /**
     * Sets the power of all drive train motors individually.
     *
     * @param lbPower Left back motor power.
     * @param rbPower Right back motor power.
     * @param lfPower Left front motor power.
     * @param rfPower Right front motor power.
     */
    public void setMotorPowers(double lbPower, double rbPower, double lfPower, double rfPower) {
        if (isMotorDisconnected()) return;
        lb.setPower(lbPower);
        rb.setPower(rbPower);
        lf.setPower(lfPower);
        rf.setPower(rfPower);
    }

    /**
     * Sets the same velocity for all drivetrain motors.
     *
     * @param velocity Velocity to set for all motors
     */
    public void setMotorVelocities(double velocity) {
        if (isMotorDisconnected()) return;
        setMotorVelocities(velocity, velocity, velocity, velocity);
    }

    /**
     * Sets individual motor velocities for the drivetrain.
     *
     * @param lbPower Left back motor velocity
     * @param rbPower Right back motor velocity
     * @param lfPower Left front motor velocity
     * @param rfPower Right front motor velocity
     */
    public void setMotorVelocities(double lbPower, double rbPower, double lfPower, double rfPower) {
        if (isMotorDisconnected()) return;
        lf.setVelocity(lfPower);
        lb.setVelocity(lbPower);
        rf.setVelocity(rfPower);
        rb.setVelocity(rbPower);
    }

    /**
     * Sets zero power behavior for all drivetrain motors.
     *
     * @param behavior The zero power behavior (BRAKE or FLOAT)
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        if (isMotorDisconnected()) return;
        lf.setZeroPowerBehavior(behavior);
        lb.setZeroPowerBehavior(behavior);
        rf.setZeroPowerBehavior(behavior);
        rb.setZeroPowerBehavior(behavior);
    }

    /**
     * Stops all drive train motors on the robot.
     */
    public void stop() {
        follower.stop();

        if (isMotorDisconnected()) return;
        setMotorPowers(0);
        setMotorVelocities(0);

        // Set target position to avoid an error
        lb.setTargetPosition(lb.getCurrentPosition());
        rb.setTargetPosition(rb.getCurrentPosition());
        lf.setTargetPosition(lf.getCurrentPosition());
        rf.setTargetPosition(rf.getCurrentPosition());

        // Turn On RUN_TO_POSITION
        setMotorModes(RUN_TO_POSITION);

        // Turn off RUN_TO_POSITION
        setMotorModes(RUN_USING_ENCODER);
    }
}
