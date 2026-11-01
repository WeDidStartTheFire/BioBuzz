package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static org.firstinspires.ftc.teamcode.constants.LaunchConstants.BALL_VEL_TO_MOTOR_VEL_COEFF;
import static org.firstinspires.ftc.teamcode.constants.LaunchConstants.BALL_VEL_TO_MOTOR_VEL_CONST;
import static org.firstinspires.ftc.teamcode.constants.LaunchConstants.launcherPIDF;
import static org.firstinspires.ftc.teamcode.constants.LaunchConstants.launcherReversePIDF;
import static org.firstinspires.ftc.teamcode.enums.Hardware.LAUNCHER_MOTOR;
import static org.firstinspires.ftc.teamcode.utils.TelemetryUtils.ErrorLevel.MEDIUM;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.PoseGetter;
import org.firstinspires.ftc.teamcode.pedro.controllers.PIDFController;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;

import java.util.concurrent.TimeUnit;

public class Launcher implements Subsystem {

    private final @Nullable DcMotorEx launcherMotor;
    private final @NonNull Timer spinningTimer;
    private boolean spinning = false;
    private @Nullable Pose lastPose;
    private @Nullable Velocity lastVel;
    private double lastGoalVel;
    private double cachedPower = 0;
    private final PIDFController pidController = new PIDFController(launcherPIDF);
    private final VoltageSensor voltageSensor;
    private final TelemetryUtils tm;
    private double cachedVel;
    private double velocityModifier;
    private final @NonNull PoseGetter poseGetter;
    private double power;

    /**
     * Initializes the launcher with hardware components.
     *
     * @param hardwareMap HardwareMap containing motor configurations
     * @param tm          TelemetryUtils instance for debugging output
     */
    public Launcher(HardwareMap hardwareMap, TelemetryUtils tm, @NonNull PoseGetter poseGetter) {
        this.tm = tm;
        this.poseGetter = poseGetter;
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        launcherMotor = HardwareInitializer.init(hardwareMap, tm, LAUNCHER_MOTOR);
        if (launcherMotor != null) {
            launcherMotor.setTargetPosition(0);
            launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherReversePIDF);
            launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        spinningTimer = new Timer();
        velocityModifier = 0;
    }

    public void increaseVelocityModifier(double increment) {
        velocityModifier += increment;
    }

    /**
     * Gets the target launch velocity at the robot's current position
     *
     * @return The target velocity in ticks/sec
     */
    public double getGoalVel() {
        return getGoalVel(poseGetter.pose(), poseGetter.vel());
    }

    /**
     * Gets the target launch velocity at a specified position
     *
     * @param pose Launch position
     * @return The target velocity in ticks/sec
     */
    public double getGoalVel(@Nullable Pose pose, @Nullable Velocity vel) {
        return 1000;
    }

    /**
     * Spins the launch motors to the optimal launch velocity at the robot's current position and
     * velocity
     */
    public void spin() {
        if (launcherMotor != null) launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double goalVel = getGoalVel();
        if (!spinning) spinningTimer.reset();
        spinning = true;
        double motorVel = getVel();
        pidController.setTarget(goalVel);
        pidController.updateFeedForwardInput(goalVel);
        pidController.updatePosition(motorVel);
        double voltage = Math.max(voltageSensor.getVoltage(), 1e-6);
        double power = pidController.calculate() * 12.0 / voltage;
        if (launcherMotor != null) launcherMotor.setPower(power);
    }

    /**
     * Gets the duration the launcher has been spinning.
     *
     * @return Duration in seconds, or 0 if not currently spinning
     */
    public double getSpinningDuration() {
        return spinning ? spinningTimer.get(TimeUnit.SECONDS) : 0;
    }

    /**
     * @return Whether the launch motors are spinning outward (>= 100 ticks/sec)
     */
    public boolean isSpinning(double vel) {
        if (launcherMotor == null) return false;
        return vel >= 100;
    }

    /**
     * Returns whether the launch motors are almost to speed (within 100 ticks/sec). WARNING: Will
     * return true even if over required speed
     *
     * @return Whether the launch motors are almost to speed (within 100 ticks/sec).
     */
    public boolean almostToSpeed(double vel) {
        if (launcherMotor == null) return false;
        return vel >= getGoalVel() - 100;
    }

    /**
     * @return Whether the launch motors are to speed (above the target launch velocity at the
     * current position)
     */
    public boolean toSpeed(double vel) {
        double motorVel = getGoalVel();
        return vel >= motorVel;
    }

    /**
     * @return Whether the launch motors spinning too fast (above the target launch velocity at the
     * current position + 50 ticks/sec)
     */
    public boolean overSpeed(double vel) {
        double motorVel = getGoalVel();
        return vel > motorVel + 50;
    }

    @Override
    public void update() {
        if (launcherMotor != null) launcherMotor.setPower(power);
    }

    /**
     * Stops the launch motors
     */
    @Override
    public void stop() {
        if (!spinning && cachedPower == 0) return;
        spinning = false;
        cachedPower = 0;
        if (launcherMotor != null) launcherMotor.setPower(0);
    }

    /**
     * Converts ball velocity to motor velocity with a scaling factor.
     *
     * @param ballVel Ball velocity in appropriate units (in/s)
     * @return Motor velocity in ticks per second
     */
    private double ballVelToMotorVel(double ballVel) {
        return BALL_VEL_TO_MOTOR_VEL_COEFF * ballVel + BALL_VEL_TO_MOTOR_VEL_CONST;
    }

    /**
     * @return Whether the launch motors are connected
     */
    public boolean isConnected() {
        return launcherMotor != null;
    }

    /**
     * Gets the velocity of the launch motors
     *
     * @return The average velocity of the launch motors. Returns only one motor velocity if one
     * motor's encoder is detected to be disconnected.
     */
    public double getVel() {
        double aRawVel = launcherMotor == null ? 0 : launcherMotor.getVelocity();
        return cachedVel = aRawVel;
    }

    public double getCachedVel() {
        return cachedVel;
    }
}
