package org.firstinspires.ftc.teamcode.pedro.localizers;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.enums.Hardware.LIMELIGHT;
import static java.lang.Math.abs;
import static java.lang.Math.hypot;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;

@Configurable
public class ComplementaryLocalizer implements Localizer {
    @Nullable
    private final Limelight3A limelight;
    @NonNull
    private Pose pose;
    @NonNull
    private Pose prevRelPose;
    @NonNull
    private Velocity vel;
    private double totalHeading;
    private final Localizer relativeLocalizer;
    private boolean invalidPose = false;
    public static double linAlpha = .97;
    public static double angAlpha = .999;

    public ComplementaryLocalizer(HardwareMap map) {
        this(map, null);
    }

    public ComplementaryLocalizer(@NonNull HardwareMap map, @Nullable Pose startPose) {
        limelight = HardwareInitializer.init(map, null, LIMELIGHT);
        if (limelight != null) {
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(0);
            limelight.start();
        }
        if (startPose == null) {
            invalidPose = true;
            startPose = Pose.zero();
        }

        relativeLocalizer = new OTOSLocalizer(map, null);
        vel = Velocity.zero();
        pose = startPose;
        prevRelPose = startPose;
        totalHeading = pose.heading();
    }

    public void setPose(@NonNull Pose setPose) {
        relativeLocalizer.setPose(setPose);
        prevRelPose = setPose;
        pose = setPose;
    }

    @Override
    public MotionState state() {
        return MotionState.ofVelocity(pose, vel);
    }

    public void update() {
        relativeLocalizer.update();

        double yawRate = relativeLocalizer.velocity().toVector2D().theta(); // imu.getRobotAngularVelocity(RADIANS).zRotationRate;

        Pose relPose = relativeLocalizer.pose();
        Pose relPoseDelta = relPose.minus(prevRelPose);
        prevRelPose = relPose;
        vel = relativeLocalizer.velocity();
        Pose lastPose = pose;
        pose = pose.plus(relPoseDelta);

        if (limelight == null) {
            totalHeading += normalizeRadians(pose.heading() - lastPose.heading());
            return;
        }

        limelight.updateRobotOrientation(toDegrees(getIMUHeading()) + 90);

        LLResult result = limelight.getLatestResult();

        Pose LLPose = null;
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose_MT2();

            if (result.getBotposeTagCount() > 0 && abs(yawRate) < toRadians(360) && hypot(vel.vx, vel.vy) < 7) {
                double angle = result.getBotpose().getOrientation().getYaw(DEGREES) - 90;
                if (angle < 0) angle += 360;
                LLPose = new Pose(botpose.getPosition().y / 0.0254 + 72,
                        -botpose.getPosition().x / 0.0254 + 72, toRadians(angle));
            }
        }
        if (LLPose != null) {
            if (invalidPose) {
                Pose3D botpose = result.getBotpose();
                if (result.getBotposeTagCount() > 0 && abs(yawRate) < toRadians(360) && hypot(vel.vx, vel.vy) < 7) {
                    invalidPose = false;
                    double angle = result.getBotpose().getOrientation().getYaw(DEGREES) - 90;
                    if (angle < 0) angle += 360;
                    LLPose = new Pose(botpose.getPosition().y / 0.0254 + 72,
                            -botpose.getPosition().x / 0.0254 + 72, toRadians(angle));
                    setPose(LLPose);
                    totalHeading = pose.heading();
                }
                return;
            }
            pose = new Pose(LLPose.x() * (1 - linAlpha) + pose.x() * linAlpha,
                    LLPose.y() * (1 - linAlpha) + pose.y() * linAlpha,
                    normalizeRadians(LLPose.heading() + angAlpha
                            * normalizeRadians(pose.heading() - LLPose.heading())));
        }
        totalHeading += normalizeRadians(pose.heading() - lastPose.heading());
    }

    @Override
    public void reset() {
        relativeLocalizer.reset();
    }

    public double getIMUHeading() {
        return pose.heading();
    }


    public boolean isNAN() {
        return Double.isNaN(pose().x()) || Double.isNaN(pose().y())
                || Double.isNaN(pose().heading());
    }
}
