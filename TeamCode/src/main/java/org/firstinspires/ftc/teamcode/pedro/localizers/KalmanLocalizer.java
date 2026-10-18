package org.firstinspires.ftc.teamcode.pedro.localizers;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.teamcode.Utils.pose3DToPose;
import static org.firstinspires.ftc.teamcode.constants.LimelightLocalizationConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.constants.LimelightLocalizationConstants.MAX_LIN_VEL;
import static org.firstinspires.ftc.teamcode.constants.LimelightLocalizationConstants.MAX_TAG_DISTANCE;
import static org.firstinspires.ftc.teamcode.constants.LimelightLocalizationConstants.MIN_MT1_TAG_AREA;
import static org.firstinspires.ftc.teamcode.constants.LimelightLocalizationConstants.MIN_MT2_TAG_AREA;
import static java.lang.Math.abs;
import static java.lang.Math.clamp;
import static java.lang.Math.hypot;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.localization.FusionLocalizer;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.localizers.OTOSConfig;
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.enums.Hive;
import org.firstinspires.ftc.teamcode.enums.HivePosition;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Limelight;

@Configurable
public class KalmanLocalizer implements Localizer {

    FusionLocalizer fusionLocalizer;
    private final Limelight limelight;
    public static Pose INITIAL_COVARIANCE = new Pose(.02, .02, toRadians(1) * toRadians(1));
    public static Pose PROCESS_VARIANCE = new Pose(1, 1, 2.5e-6);
    private static final Pose MEASUREMENT_VARIANCE =
            new Pose(81, 81, toRadians(20) * toRadians(20));
    public static double MAX_MT2_LIN_VARIANCE = 81;
    public static double MAX_MT1_LIN_VARIANCE = 144;
    public static double MAX_LL_ANG_VARIANCE = toRadians(20) * toRadians(20);
    public static int MAX_TAG_VAR_REDUCTION = 4;
    private static final int BUFFER_SIZE = 100;
    private boolean invalidPose = false;
    private int currentPipeline;


    public KalmanLocalizer(HardwareMap map, OTOSConfig config) {
        this(map, null, config);
    }

    public KalmanLocalizer(@NonNull HardwareMap map, @Nullable Pose startPose,
                           @NonNull OTOSConfig config) {
        limelight = new Limelight(map, null);
        if (limelight.limelight != null) {
            limelight.limelight.setPollRateHz(100);
            limelight.limelight.pipelineSwitch(0);
        }
        limelight.start();
        if (startPose == null) {
            startPose = Pose.zero();
            invalidPose = true;
        }

        Localizer relativeLocalizer = new OTOSLocalizer(map, config);
        fusionLocalizer = new FusionLocalizer(relativeLocalizer, INITIAL_COVARIANCE,
                PROCESS_VARIANCE, MEASUREMENT_VARIANCE, BUFFER_SIZE);
        fusionLocalizer.setStartPose(startPose);
    }

    public void correctPose(@NonNull Pose correctedPose) {
        fusionLocalizer.correctPose(correctedPose);
        invalidPose = false;
    }

    public void setPose(@NonNull Pose setPose) {
        fusionLocalizer.setPose(setPose);
        invalidPose = false;
    }

    @Override
    public MotionState state() {
        return fusionLocalizer.state();
    }

    public void update() {
        fusionLocalizer.update();
        Pose pose = fusionLocalizer.pose();
        Velocity vel = fusionLocalizer.velocity();

        if (limelight.limelight == null) return;

        switchLimelightPipeline();
        limelight.limelight.updateRobotOrientation(toDegrees(pose.heading()) + 90);
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid() || abs(vel.omega) > MAX_ANG_VEL ||
                hypot(vel.vx, vel.vy) > MAX_LIN_VEL || result.getBotposeTagCount() <= 0 ||
                result.getTa() < MIN_MT2_TAG_AREA) return;

        Pose LLPose = null;
        boolean useMetaTag2 = false;
        int tagsCounted = 0;
        if (hivesMatchFMap()) {
            Pose3D botpose = result.getBotpose_MT2();
            double angle = result.getBotpose().getOrientation().getYaw(DEGREES) - 90;
            if (angle < 0) angle += 360;
            LLPose = pose3DToPose(botpose).withHeading(toRadians(angle));
            useMetaTag2 = true;
        } else {
            Pose totalPos = Pose.zero();
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                Pose botpose = pose3DToPose(fiducial.getRobotPoseFieldSpace());
                double distance =
                        pose3DToPose(fiducial.getTargetPoseCameraSpace()).distance(Pose.zero());
                if (tagMatchesFMap(fiducial) && botpose.distance(pose) < 12 &&
                        fiducial.getTargetArea() > MIN_MT1_TAG_AREA &&
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
        if (invalidPose) {
            Pose3D botpose = result.getBotpose();
            LLPose = useMetaTag2 ? pose3DToPose(botpose) : LLPose;
            correctPose(LLPose);
            return;
        }
        int numTags = useMetaTag2 ? result.getBotposeTagCount() : tagsCounted;
        numTags = clamp(numTags, 0, MAX_TAG_VAR_REDUCTION);
        if (numTags == 0) return;
        double linVar = useMetaTag2 ? MAX_MT2_LIN_VARIANCE / numTags
                : MAX_MT1_LIN_VARIANCE / numTags;
        double angVar = MAX_LL_ANG_VARIANCE / numTags;
        fusionLocalizer.addMeasurement(LLPose, result.getControlHubTimeStampNanos(),
                new Pose(linVar, linVar, angVar));
    }

    @Override
    public void reset() {
        fusionLocalizer.reset();
    }


    public void switchLimelightPipeline() {
        if (limelight.limelight == null) return;

        Pose pose = fusionLocalizer.pose();

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
