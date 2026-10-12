package org.firstinspires.ftc.teamcode.pedro.localizers;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.teamcode.Utils.pose3DToPose;
import static org.firstinspires.ftc.teamcode.constants.DrivetrainConstants.IMU_PARAMS;
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
import com.pedropathing.revhub.localizers.OTOSConfig;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.enums.Hardware;
import org.firstinspires.ftc.teamcode.enums.Hive;
import org.firstinspires.ftc.teamcode.enums.HivePosition;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Limelight;

@Configurable
public class LimelightLocalizer implements Localizer {
    private final Limelight limelight;
    @Nullable
    private final IMU imu;
    @NonNull
    private Pose pose;
    @NonNull
    private final Velocity vel;
    private int currentPipeline;

    public LimelightLocalizer(HardwareMap map, OTOSConfig config) {
        this(map, null, config);
    }

    public LimelightLocalizer(@NonNull HardwareMap map, @Nullable Pose startPose,
                              @NonNull OTOSConfig config) {
        imu = HardwareInitializer.init(map, null, Hardware.IMU);
        if (imu != null) {
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
            if (!imu.initialize(IMU_PARAMS))
                throw new RuntimeException("IMU initialization failed");
            imu.resetYaw();
        }

        limelight = new Limelight(map, null);
        if (limelight.limelight != null) {
            limelight.limelight.setPollRateHz(100);
            limelight.limelight.pipelineSwitch(0);
        }
        limelight.start();
        if (startPose == null) startPose = Pose.zero();

        vel = Velocity.zero();
        pose = startPose;
    }

    public void setPose(@NonNull Pose setPose) {
        pose = setPose;
    }

    @Override
    public MotionState state() {
        return MotionState.ofVelocity(pose, vel);
    }

    public void update() {
        if (limelight.limelight == null) return;

        double yawRate = imu == null ? 0 : imu.getRobotAngularVelocity(RADIANS).zRotationRate;

        switchLimelightPipeline();

        limelight.limelight.updateRobotOrientation(toDegrees(pose.heading()) + 90);

        LLResult result = limelight.getLatestResult();

        Pose LLPose = null;
        int tagsCounted = 0;
        if (result != null && result.isValid()) {
            if (hivesMatchFMap()) {
                Pose3D botpose = result.getBotpose_MT2();

                if (result.getBotposeTagCount() > 0 && abs(yawRate) < toRadians(360) && hypot(vel.vx, vel.vy) < 7) {
                    double angle = result.getBotpose().getOrientation().getYaw(DEGREES) - 90;
                    if (angle < 0) angle += 360;
                    LLPose = pose3DToPose(botpose).withHeading(toRadians(angle));
                }
            } else {
                Pose totalPos = Pose.zero();
                for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                    if (tagMatchesFMap(fiducial)) {
                        Pose botpose = pose3DToPose(fiducial.getRobotPoseFieldSpace());
                        if (botpose.distance(pose) < 12) {
                            totalPos = totalPos.plus(botpose);
                            tagsCounted++;
                        }
                    }
                }
                if (tagsCounted > 0) {
                    totalPos = totalPos.div(tagsCounted);
                    LLPose = totalPos;
                }
            }
        }
        if (LLPose != null) pose = LLPose;
    }

    @Override
    public void reset() {
    }


    public void switchLimelightPipeline() {
        if (limelight.limelight == null) return;

        if (pose.y() > 96) {
            if (currentPipeline != 0) {
                limelight.limelight.pipelineSwitch(0);
                currentPipeline = 0;
            }
        } else if (pose.y() < 48) {
            if (currentPipeline != 1) {
                limelight.limelight.pipelineSwitch(1);
                currentPipeline = 1;
            }
        } else if (pose.y() < 95 && pose.y() > 49) {
            if (pose.heading() > toRadians(5) && pose.heading() < toRadians(175)) {
                if (currentPipeline != 1) {
                    limelight.limelight.pipelineSwitch(1);
                    currentPipeline = 1;
                }
            } else {
                if (pose.heading() < toRadians(355) && pose.heading() > toRadians(185)) {
                    if (currentPipeline != 0) {
                        limelight.limelight.pipelineSwitch(0);
                        currentPipeline = 0;
                    }
                }
            }
        }
    }

    public boolean hivesMatchFMap() {
        if ((limelight.getHivePosition(Color.RED) == HivePosition.UNKNOWN &&
                limelight.getHivePosition(Color.BLUE) == HivePosition.UNKNOWN) ||
                limelight.getHivePosition(Color.RED) == HivePosition.TRANSITIONING ||
                limelight.getHivePosition(Color.BLUE) == HivePosition.TRANSITIONING) {
            return false;
        }
        if (currentPipeline == 1 && ((limelight.getHivePosition(Color.RED) == HivePosition.AUDIENCE && // If both hives are audience
                limelight.getHivePosition(Color.BLUE) == HivePosition.AUDIENCE) ||
                (limelight.getHivePosition(Color.RED) == HivePosition.AUDIENCE && // If Red is Audience and Blue is unknown
                        limelight.getHivePosition(Color.BLUE) == HivePosition.UNKNOWN) ||
                (limelight.getHivePosition(Color.RED) == HivePosition.UNKNOWN && // If Blue is Audience and Red is unknown
                        limelight.getHivePosition(Color.BLUE) == HivePosition.AUDIENCE))) {
            return true;
        }
        return currentPipeline == 0 && ((limelight.getHivePosition(Color.RED) == HivePosition.STAGE && // If both hives are Stage
                limelight.getHivePosition(Color.BLUE) == HivePosition.STAGE) ||
                (limelight.getHivePosition(Color.RED) == HivePosition.STAGE && // If Red is Stage and Blue is unknown
                        limelight.getHivePosition(Color.BLUE) == HivePosition.UNKNOWN) ||
                (limelight.getHivePosition(Color.RED) == HivePosition.UNKNOWN && // If Blue is Stage and Red is unknown
                        limelight.getHivePosition(Color.BLUE) == HivePosition.STAGE));
    }

    public boolean tagMatchesFMap(LLResultTypes.FiducialResult fiducial) {
        Color color = Hive.matchTagToColor(fiducial.getFiducialId());
        if (limelight.getHivePosition(color) == HivePosition.AUDIENCE && currentPipeline == 1) {
            return true;
        }
        return limelight.getHivePosition(color) == HivePosition.STAGE && currentPipeline == 0;
    }
}
