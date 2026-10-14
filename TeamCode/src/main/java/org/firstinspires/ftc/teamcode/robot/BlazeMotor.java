package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.enums.Hardware;

import dev.anygeneric.blazeftc.BlazeFTC;

public class BlazeMotor implements DcMotorEx {
    @Nullable
    private final DcMotorEx device;
    public final Hardware hardware;
    private double power;

    public BlazeMotor(@NonNull HardwareMap hardwareMap, @Nullable TelemetryUtils tm,
                      @NonNull Hardware hardware) {
        assert hardware.classOrInterface == DcMotorEx.class;
        device = HardwareInitializer.init(hardwareMap, tm, hardware);
        this.hardware = hardware;
    }

    public boolean isInitialized() {
        return device != null;
    }

    public void writePower() {
        writePower(power);
    }

    public void setPower(double power) {
        this.power = power;
    }

    public double getPower() {
        return power;
    }

    public void writePower(double power) {
        BlazeFTC.setMotorPower(hardware.hub.id(), hardware.port, power);
    }

    @Override
    public void setDirection(Direction direction) {
        if (device != null) device.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return device == null ? Direction.FORWARD : device.getDirection();
    }

    @Override
    public void setMotorEnable() {
        if (device != null) device.setMotorEnable();
    }

    @Override
    public void setMotorDisable() {
        if (device != null) device.setMotorDisable();
    }

    @Override
    public boolean isMotorEnabled() {
        return device != null && device.isMotorEnabled();
    }

    @Override
    public void setVelocity(double angularRate) {
        if (device != null) device.setVelocity(angularRate);
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        if (device != null) device.setVelocity(angularRate, unit);
    }

    @Override
    public double getVelocity() {
        return device != null ? device.getVelocity() : 0;
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return device != null ? device.getVelocity(unit) : 0;
    }

    @Override
    @Deprecated
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        if (device != null) device.setPIDCoefficients(mode, pidCoefficients);
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients)
            throws UnsupportedOperationException {
        if (device != null) device.setPIDFCoefficients(mode, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        if (device != null) device.setVelocityPIDFCoefficients(p, i, d, f);
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        if (device != null) device.setPositionPIDFCoefficients(p);
    }

    @Override
    @Deprecated
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return device == null ? new PIDCoefficients() : device.getPIDCoefficients(mode);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return device == null ? new PIDFCoefficients() : device.getPIDFCoefficients(mode);
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        if (device != null) device.setTargetPositionTolerance(tolerance);
    }

    @Override
    public int getTargetPositionTolerance() {
        return device == null ? 0 : device.getTargetPositionTolerance();
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return device == null ? 0 : device.getCurrent(unit);
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return device == null ? 0 : device.getCurrentAlert(unit);
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        if (device != null) device.setCurrentAlert(current, unit);
    }

    @Override
    public boolean isOverCurrent() {
        return device != null && device.isOverCurrent();
    }

    @Override
    @Nullable
    public MotorConfigurationType getMotorType() {
        return device == null ? null : device.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        if (device != null) device.setMotorType(motorType);
    }

    @Override
    @Nullable
    public DcMotorController getController() {
        return device == null ? null : device.getController();
    }

    @Override
    public int getPortNumber() {
        return device == null ? -1 : device.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        if (device != null) device.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return device == null ? ZeroPowerBehavior.FLOAT : device.getZeroPowerBehavior();
    }

    @Override
    @Deprecated
    public void setPowerFloat() {
        if (device != null) device.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return device != null && device.getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        if (device != null) device.setTargetPosition(position);
    }

    @Override
    public int getTargetPosition() {
        return device == null ? 0 : device.getTargetPosition();
    }

    @Override
    public boolean isBusy() {
        return device != null && device.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return device == null ? 0 : device.getCurrentPosition();
    }

    @Override
    public void setMode(RunMode mode) {
        if (device != null) device.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return device == null ? RunMode.RUN_WITHOUT_ENCODER : device.getMode();
    }

    @Override
    public Manufacturer getManufacturer() {
        return device == null ? Manufacturer.Unknown : device.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return device == null ? "" : device.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return device == null ? "" : device.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return device == null ? 0 : device.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        if (device != null) device.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        if (device != null) device.close();
    }
}
