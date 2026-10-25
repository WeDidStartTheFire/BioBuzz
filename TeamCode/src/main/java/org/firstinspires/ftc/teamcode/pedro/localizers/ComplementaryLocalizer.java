package org.firstinspires.ftc.teamcode.pedro.localizers;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.teamcode.constants.LocalizationConstants.LERP_END_WEIGHT;
import static org.firstinspires.ftc.teamcode.constants.LocalizationConstants.LERP_START;
import static org.firstinspires.ftc.teamcode.constants.LocalizationConstants.MAX_AMBIGUITY;
import static org.firstinspires.ftc.teamcode.constants.LocalizationConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.constants.LocalizationConstants.MAX_DIST;
import static org.firstinspires.ftc.teamcode.constants.LocalizationConstants.MAX_LIN_VEL;
import static org.firstinspires.ftc.teamcode.constants.LocalizationConstants.MAX_TAG_DISTANCE;
import static org.firstinspires.ftc.teamcode.utils.Utils.lerp;
import static org.firstinspires.ftc.teamcode.utils.Utils.pose3DToPose;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.clamp;
import static java.lang.Math.hypot;
import static java.lang.Math.sqrt;
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
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.enums.Hive;
import org.firstinspires.ftc.teamcode.enums.HivePosition;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Limelight;
import org.firstinspires.ftc.teamcode.utils.LLUtils;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.json.JSONArray;

@Configurable
public class ComplementaryLocalizer extends LoggingLocalizer {
    private final Limelight limelight;
    @NonNull
    private Pose pose;
    @NonNull
    private Pose prevRelPose;
    @NonNull
    private Velocity vel;
    private final Localizer relativeLocalizer;
    private boolean invalidPose = false;
    private int currentPipeline;
    public static double MAX_TAG_ALPHA_REDUCTION = 4;
    public static double MIN_MT2_LIN_ALPHA = .95;
    public static double MIN_MT1_LIN_ALPHA = .97;
    public static boolean USE_CONSTANT_LIN_ALPHA = true;
    public static double CONSTANT_LIN_ALPHA = .96;
    private long lastMeasurmentTime = 0;

    public ComplementaryLocalizer(HardwareMap map, OTOSConfig config) {
        this(map, null, config);
    }

    public ComplementaryLocalizer(@NonNull HardwareMap map, @Nullable Pose startPose,
                                  @NonNull OTOSConfig config) {
        limelight = new Limelight(map, null);
        if (limelight.limelight != null) {
            limelight.limelight.setPollRateHz(100);
            limelight.limelight.pipelineSwitch(0);
        }
        limelight.start();
        if (startPose == null) {
            invalidPose = true;
            startPose = Pose.zero();
        }

        relativeLocalizer = new OTOSLocalizer(map, config);
        relativeLocalizer.setPose(startPose);
        vel = Velocity.zero();
        pose = startPose;
        prevRelPose = startPose;
    }

    public void setPose(@Nullable Pose setPose) {
        if (setPose == null) return;
        relativeLocalizer.setPose(setPose);
        prevRelPose = setPose;
        pose = setPose;
        invalidPose = false;
    }

    @Override
    public MotionState state() {
        return MotionState.ofVelocity(pose, vel);
    }

    @Override
    public void initializeLogging() {
        columns = new String[]{"timestamp", "ll timestamp", "initial pose x", "initial pose y",
                "initial pose h", "initial vel x", "initial vel y", "initial vel h", "otos pose x",
                "otos pose y", "otos pose h", "corrected pose x", "corrected pose y",
                "corrected pose h", "invalid pose", "pipeline", "valid result", "angular velocity",
                "linear velocity", "tag count", "rejected", "hives match", "avg dist",
                "weighted dist", "avg ambig (all)", "valid tags", "total weight", "ll pose x",
                "ll pose y", "ll pose h", "lin alpha"};
        super.initializeLogging();
    }

    public void update() {
        resetLog();
        addLog("timestamp", System.nanoTime());

        relativeLocalizer.update();
        Pose relPose = relativeLocalizer.pose();
        Pose relPoseDelta = relPose.minus(prevRelPose);
        prevRelPose = relPose;
        vel = relativeLocalizer.velocity();
        pose = pose.plus(relPoseDelta);

        addLog("initial pose", pose);
        addLog("initial vel", vel);
        addLog("otos pose", relativeLocalizer.pose());
        if (tm != null) tm.drawRobot(relativeLocalizer.pose(), false);
        addLog("invalid pose", invalidPose);

        if (limelight.limelight == null) return;

        switchLimelightPipeline();
        limelight.limelight.updateRobotOrientation(toDegrees(pose.heading()) + 90);
        LLResult result = limelight.getLatestResult();

        addLog("ll timestamp", result != null ? result.getControlHubTimeStampNanos() : 0);
        addLog("pipeline", currentPipeline);
        addLog("valid result", result != null && result.isValid());
        addLog("angular velocity", abs(vel.omega));
        addLog("linear velocity", hypot(vel.vx, vel.vy));
        addLog("tag count", result != null && result.isValid() ?
                result.getBotposeTagCount() : 0);


        if (result == null || !result.isValid() || abs(vel.omega) > MAX_ANG_VEL ||
                hypot(vel.vx, vel.vy) > MAX_LIN_VEL || result.getBotposeTagCount() <= 0) {
            addLog("rejected", true);
            return;
        }

        if (tm == null && !logging && lastMeasurmentTime == result.getControlHubTimeStampNanos())
            return;

        addLog("rejected", false);
        addLog("hives match", hivesMatchFMap());


        Pose LLPose = null;
        boolean useAll = false;
        int tagsCounted = 0;
        double totalWeight = 0;
        if (hivesMatchFMap()) {
            Pose3D botpose = invalidPose ? result.getBotpose() : result.getBotpose_MT2();
            double angle = result.getBotpose().getOrientation().getYaw(DEGREES) - 90;
            if (angle < 0) angle += 360;
            LLPose = pose3DToPose(botpose).withHeading(toRadians(angle));
            useAll = true;
        } else {
            Pose weightedTotal = Pose.zero();
            Pose totalPose = Pose.zero();
            double totalAmbiguity = 0;
            JSONArray fiducialArray = LLUtils.getFiducialArray(result);
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                Pose3D botpose3d = invalidPose ? fiducial.getRobotPoseFieldSpace() :
                        LLUtils.getRobotPoseFieldSpace_MT2(fiducialArray, fiducial);
                if (botpose3d == null) continue;
                double heading = fiducial.getRobotPoseFieldSpace().getOrientation().getYaw(RADIANS)
                        - PI * 0.5;
                if (heading < 0) heading += 2 * PI;
                Pose botpose = pose3DToPose(botpose3d).withHeading(heading);
                double tagDist = pose3DToPose(fiducial.getTargetPoseCameraSpace())
                        .distance(new Pose(72, 72));
                double distFromCurrentEstimate = invalidPose ? 0 : botpose.distance(pose);
                double ambiguity = LLUtils.getAmbiguity(fiducialArray, fiducial);
                totalAmbiguity += ambiguity;
                if (tagMatchesFMap(fiducial) && distFromCurrentEstimate <= MAX_DIST &&
                        tagDist <= MAX_TAG_DISTANCE && ambiguity <= MAX_AMBIGUITY) {
                    double weight = clamp(lerp((distFromCurrentEstimate - LERP_START) /
                            (MAX_DIST - LERP_START), 1, LERP_END_WEIGHT), LERP_END_WEIGHT, 1);
                    weightedTotal = weightedTotal.plus(botpose.times(weight));
                    if (tm != null || logging) totalPose = totalPose.plus(botpose);
                    totalWeight += weight;
                    tagsCounted++;
                }
            }
            if (tagsCounted > 0) {
                weightedTotal = weightedTotal.div(totalWeight);
                LLPose = weightedTotal;
                if (tm == null || logging) {
                    Pose avgPose = totalPose.div(tagsCounted);
                    addLog("avg dist", pose.distance(avgPose));
                    addLog("weighted dist", pose.distance(LLPose));
                    addLog("avg ambig (all)", totalAmbiguity / result.getBotposeTagCount());
                }
            }
        }

        int numTags = useAll ? result.getBotposeTagCount() : tagsCounted;
        addLog("valid tags", numTags);
        addLog("total weight", totalWeight);

        if (LLPose == null) return;
        addLog("ll pose", LLPose);
        if (tm != null) tm.drawRobot(LLPose, false);

        if (invalidPose) {
            setPose(LLPose);
            lastMeasurmentTime = result.getControlHubTimeStampNanos();
            return;
        }

        double linAlpha = USE_CONSTANT_LIN_ALPHA ? CONSTANT_LIN_ALPHA : useAll ?
                clamp(Utils.lerp(sqrt(result.getBotposeTagCount() / MAX_TAG_ALPHA_REDUCTION), 1,
                        MIN_MT2_LIN_ALPHA), MIN_MT2_LIN_ALPHA, 1) :
                clamp(Utils.lerp(sqrt(tagsCounted / MAX_TAG_ALPHA_REDUCTION), 1,
                        MIN_MT1_LIN_ALPHA), MIN_MT1_LIN_ALPHA, 1);

        addLog("lin alpha", linAlpha);
        if (invalidPose) {
            setPose(LLPose);
            lastMeasurmentTime = result.getControlHubTimeStampNanos();
            return;
        }
        if (result.getControlHubTimeStampNanos() == lastMeasurmentTime) return;

        pose = new Pose(LLPose.x() * (1 - linAlpha) + pose.x() * linAlpha,
                LLPose.y() * (1 - linAlpha) + pose.y() * linAlpha, pose.heading());
        addLog("corrected pose", pose);
        lastMeasurmentTime = result.getControlHubTimeStampNanos();
    }

    @Override
    public void reset() {
        relativeLocalizer.reset();
        pose = relativeLocalizer.pose();
        prevRelPose = pose;
        vel = Velocity.zero();
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
