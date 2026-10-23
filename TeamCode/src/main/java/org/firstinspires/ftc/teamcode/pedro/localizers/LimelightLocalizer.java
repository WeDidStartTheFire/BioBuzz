package org.firstinspires.ftc.teamcode.pedro.localizers;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
import static org.firstinspires.ftc.teamcode.constants.DrivetrainConstants.IMU_PARAMS;
import static org.firstinspires.ftc.teamcode.constants.LocalizationConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.constants.LocalizationConstants.MAX_LIN_VEL;
import static org.firstinspires.ftc.teamcode.constants.LocalizationConstants.MAX_TAG_DISTANCE;
import static org.firstinspires.ftc.teamcode.utils.Utils.pose3DToPose;
import static java.lang.Math.abs;
import static java.lang.Math.hypot;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.enums.Hardware;
import org.firstinspires.ftc.teamcode.enums.Hive;
import org.firstinspires.ftc.teamcode.enums.HivePosition;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Limelight;

public class LimelightLocalizer implements Localizer {
    private final Limelight limelight;
    @Nullable
    private final IMU imu;
    @NonNull
    private Pose pose;
    @NonNull
    private Velocity vel;
    private int currentPipeline;
    private long lastUpdate;
    private double imuOffset;

    public LimelightLocalizer(HardwareMap map) {
        this(map, null);
    }

    public LimelightLocalizer(@NonNull HardwareMap map, @Nullable Pose startPose) {
        imu = HardwareInitializer.init(map, null, Hardware.IMU);
        if (imu != null) {
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

        setPose(startPose);
    }

    public void setPose(@NonNull Pose setPose) {
        vel = Velocity.zero();
        pose = setPose;
        imuOffset = pose.heading() - (imu == null ? 0 :
                imu.getRobotOrientation(INTRINSIC, ZYX, RADIANS).firstAngle);
    }

    @Override
    public MotionState state() {
        return MotionState.ofVelocity(pose, vel);
    }

    public void update() {
        if (limelight.limelight == null) return;

        double yawRate = imu == null ? 0 : imu.getRobotAngularVelocity(RADIANS).zRotationRate;
        double yaw = imu == null ? 0 :
                imu.getRobotOrientation(INTRINSIC, ZYX, RADIANS).firstAngle + imuOffset;

        switchLimelightPipeline();
        limelight.limelight.updateRobotOrientation(toDegrees(yaw) + 90);
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid() || abs(yawRate) > MAX_ANG_VEL ||
                hypot(vel.vx, vel.vy) > MAX_LIN_VEL || result.getBotposeTagCount() <= 0) return;


        Pose LLPose = null;
        int tagsCounted = 0;
        if (hivesMatchFMap()) {
            Pose3D botpose = result.getBotpose_MT2();
            double angle = result.getBotpose().getOrientation().getYaw(DEGREES) - 90;
            if (angle < 0) angle += 360;
            LLPose = pose3DToPose(botpose).withHeading(toRadians(angle));
        } else {
            Pose totalPos = Pose.zero();
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                Pose botpose = pose3DToPose(fiducial.getRobotPoseFieldSpace());
                double distance =
                        pose3DToPose(fiducial.getTargetPoseCameraSpace()).distance(Pose.zero());
                if (tagMatchesFMap(fiducial) && botpose.distance(pose) < 12 &&
                        distance < MAX_TAG_DISTANCE) {
                    totalPos = totalPos.plus(botpose);
                    tagsCounted++;
                }
            }
            if (tagsCounted > 0) {
                totalPos = totalPos.div(tagsCounted);
                LLPose = totalPos;
            }
        }

        if (LLPose == null) return;
        double dt = (result.getControlHubTimeStampNanos() - lastUpdate) / 1e9;
        Pose dPose = LLPose.minus(pose);
        pose = LLPose;
        vel = new Velocity(dPose.x() / dt, dPose.y() / dt, dPose.heading() / dt);
        lastUpdate = result.getControlHubTimeStampNanos();
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
