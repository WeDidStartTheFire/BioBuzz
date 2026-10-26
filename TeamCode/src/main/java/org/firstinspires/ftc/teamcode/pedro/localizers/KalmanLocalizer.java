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
import static org.firstinspires.ftc.teamcode.utils.Utils.angleAverage;
import static org.firstinspires.ftc.teamcode.utils.Utils.lerp;
import static org.firstinspires.ftc.teamcode.utils.Utils.pose3DToPose;
import static org.firstinspires.ftc.teamcode.utils.Utils.weightedAngleAverage;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.clamp;
import static java.lang.Math.hypot;
import static java.lang.Math.max;
import static java.lang.Math.pow;
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
import org.firstinspires.ftc.teamcode.utils.LLUtils;
import org.json.JSONArray;

import java.util.ArrayList;

@Configurable
public class KalmanLocalizer extends LoggingLocalizer {

    FusionLocalizer fusionLocalizer;
    private final Limelight limelight;
    public static Pose INITIAL_COVARIANCE = new Pose(.02, .02, toRadians(1) * toRadians(1));
    public static Pose PROCESS_VARIANCE = new Pose(0.2, 0.2, 1.1984518791e-5);
    private static final Pose MEASUREMENT_VARIANCE =
            new Pose(81, 81, toRadians(20) * toRadians(20));
    public static double MAX_MT2_LIN_VAR = 81;
    public static double MAX_FILTERED_LIN_VAR = 144;
    public static double MAX_LL_ANG_VARIANCE = toRadians(20) * toRadians(20);
    public static int MAX_TAG_VAR_REDUCTION = 4;
    public static boolean USE_MEASUREMENT_VARIANCE = true;
    public static double X_VAR_SCALAR = 1, Y_VAR_SCALAR = 1, ANG_VAR_SCALAR = 1;
    private static final int BUFFER_SIZE = 100;
    private boolean invalidPose = false;
    private int currentPipeline;
    private long lastMeasurmentTime;
    private final Localizer relativeLocalizer;


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

        relativeLocalizer = new OTOSLocalizer(map, config);
        fusionLocalizer = new FusionLocalizer(relativeLocalizer, INITIAL_COVARIANCE,
                PROCESS_VARIANCE, MEASUREMENT_VARIANCE, BUFFER_SIZE);
        fusionLocalizer.setStartPose(startPose);
    }

    private void reinitialize(@NonNull Pose pose) {
        fusionLocalizer = new FusionLocalizer(relativeLocalizer, MEASUREMENT_VARIANCE,
                PROCESS_VARIANCE, MEASUREMENT_VARIANCE, BUFFER_SIZE);
        fusionLocalizer.setStartPose(pose);
        invalidPose = false;
        lastMeasurmentTime = 0;
    }

    public void setPose(@Nullable Pose setPose) {
        if (setPose == null) return;
        fusionLocalizer.setPose(setPose);
        invalidPose = false;
    }

    @Override
    public void initializeLogging() {
        columns = new String[]{"timestamp", "ll timestamp", "initial pose x", "initial pose y",
                "initial pose h", "initial vel x", "initial vel y", "initial vel h", "otos pose x",
                "otos pose y", "otos pose h", "corrected pose x", "corrected pose y",
                "corrected pose h", "corrected vel x", "corrected vel y", "corrected vel h",
                "invalid pose", "pipeline", "valid result", "angular velocity", "linear velocity",
                "tag count", "rejected", "hives match", "avg dist", "weighted dist",
                "avg ambig (all)", "valid tags", "total weight", "ll pose x", "ll pose y",
                "ll pose h", "lin var", "ang var", "x var", "y var", "head var"};
        super.initializeLogging();
    }

    @Override
    public MotionState state() {
        return fusionLocalizer.state();
    }

    public void update() {
        resetLog();
        addLog("timestamp", System.nanoTime());

        fusionLocalizer.update();
        Pose pose = fusionLocalizer.pose();
        Velocity vel = fusionLocalizer.velocity();

        addLog("initial pose", pose);
        addLog("initial vel", vel);
        addLog("otos pose", relativeLocalizer.pose());
        if (tm != null) tm.drawRobot(relativeLocalizer.pose(), false);
        addLog("invalid pose", invalidPose);

        if (limelight.limelight == null) return;

        switchLimelightPipeline();
        limelight.limelight.updateRobotOrientation(toDegrees(pose.heading()) + 90);
        LLResult result = limelight.getLatestResult();
        long resultTimestamp = result == null ? 0 : System.nanoTime() - 1_000_000L *
                (System.currentTimeMillis() - result.getControlHubTimeStamp());

        addLog("ll timestamp", resultTimestamp);
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

        if (tm == null && !logging && lastMeasurmentTime == resultTimestamp)
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
            ArrayList<Double> headings = new ArrayList<>();
            ArrayList<Double> weights = new ArrayList<>();
            double totalAmbiguity = 0;
            JSONArray fiducialArray = LLUtils.getFiducialArray(result);
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                Pose3D botpose3d = invalidPose ? fiducial.getRobotPoseFieldSpace() :
                        LLUtils.getRobotPoseFieldSpace_MT2(fiducialArray, fiducial);
                if (botpose3d == null) continue;
                double heading = fiducial.getRobotPoseFieldSpace().getOrientation().getYaw(RADIANS)
                        - PI * 0.5;
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
                    headings.add(heading);
                    weights.add(weight);
                    totalWeight += weight;
                    tagsCounted++;
                }
            }
            if (tagsCounted > 0) {
                double weightedHeading = weightedAngleAverage(headings, weights);
                weightedTotal = weightedTotal.div(totalWeight).withHeading(weightedHeading);
                LLPose = weightedTotal;
                if (tm != null || logging) {
                    Pose avgPose = totalPose.div(tagsCounted).withHeading(angleAverage(headings));
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
        if (invalidPose && !logging && tm == null) {
            reinitialize(LLPose);
            lastMeasurmentTime = resultTimestamp;
            return;
        }
        addLog("ll pose", LLPose);
        if (tm != null) tm.drawRobot(LLPose, false);

        numTags = clamp(numTags, 0, MAX_TAG_VAR_REDUCTION);
        if (numTags == 0) return;
        double linVar = useAll ? MAX_MT2_LIN_VAR / numTags : MAX_FILTERED_LIN_VAR / numTags;
        double angVar = MAX_LL_ANG_VARIANCE / numTags;
        double[] stddevs = result.getStddevMt2();
        double xVar = max(1, pow(stddevs[0] * 39.37, 2)) * X_VAR_SCALAR;
        double yVar = max(1, pow(stddevs[1] * 39.37, 2)) * Y_VAR_SCALAR;
        double hVar = max(pow(toRadians(10), 2), pow(toRadians(result.getStddevMt1()[5]), 2)) *
                ANG_VAR_SCALAR;

        addLog("lin var", linVar);
        addLog("ang var", angVar);
        addLog("x var", xVar / X_VAR_SCALAR);
        addLog("y var", yVar / Y_VAR_SCALAR);
        addLog("head var", hVar / ANG_VAR_SCALAR);

        if (invalidPose) {
            reinitialize(LLPose);
            lastMeasurmentTime = resultTimestamp;
            return;
        }
        if (resultTimestamp == lastMeasurmentTime) return;

        fusionLocalizer.addMeasurement(LLPose, resultTimestamp,
                USE_MEASUREMENT_VARIANCE ? new Pose(xVar, yVar, hVar) :
                        new Pose(linVar, linVar, angVar));
        addLog("corrected pose", fusionLocalizer.pose());
        addLog("corrected vel", fusionLocalizer.velocity());
        lastMeasurmentTime = resultTimestamp;
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
