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
import static org.firstinspires.ftc.teamcode.utils.TelemetryUtils.PrintLevel.DEBUG;
import static org.firstinspires.ftc.teamcode.utils.Utils.lerp;
import static org.firstinspires.ftc.teamcode.utils.Utils.pose3DToPose;
import static java.lang.Math.abs;
import static java.lang.Math.clamp;
import static java.lang.Math.hypot;
import static java.lang.Math.max;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
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
import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;

import java.util.Locale;

@Configurable
public class KalmanLocalizer implements Localizer {

    FusionLocalizer fusionLocalizer;
    private final Limelight limelight;
    public static Pose INITIAL_COVARIANCE = new Pose(.02, .02, toRadians(1) * toRadians(1));
    public static Pose PROCESS_VARIANCE = new Pose(1, 1, 2.5e-6);
    private static final Pose MEASUREMENT_VARIANCE =
            new Pose(81, 81, toRadians(20) * toRadians(20));
    public static double MAX_MT2_LIN_VAR = 81;
    public static double MAX_FILTERED_LIN_VAR = 144;
    public static double MAX_LL_ANG_VARIANCE = toRadians(20) * toRadians(20);
    public static int MAX_TAG_VAR_REDUCTION = 4;
    public static boolean USE_MEASUREMENT_VARIANCE = true;
    private static final int BUFFER_SIZE = 100;
    private boolean invalidPose = false;
    private int currentPipeline;
    @Nullable
    private TelemetryUtils tm;
    private long lastMeasurmentTime;
    private StringBuilder log = new StringBuilder();
    private boolean logging = false;
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

    public void setPose(@NonNull Pose setPose) {
        fusionLocalizer.setPose(setPose);
        invalidPose = false;
    }

    public void enableTelemetry(TelemetryUtils tm) {
        this.tm = tm;
    }

    public void enableLogging() {
        this.logging = true;
    }

    public String getLog() {
        return log.toString();
    }

    private void addLog(@NonNull Object caption, @NonNull Object content) {
        if (tm != null) tm.print(caption, content, DEBUG);
        if (logging) {
            if (content instanceof Pose) {
                log.append(String.format(Locale.US, "%s,%s,%s,",
                        ((Pose) content).x(), ((Pose) content).y(), ((Pose) content).heading()));
            } else if (content instanceof Velocity) {
                log.append(String.format(Locale.US, "%s,%s,%s,", ((Velocity) content).vx,
                        ((Velocity) content).vy, ((Velocity) content).omega));
            } else log.append(String.format(Locale.US, "%s,", content));
        }
    }

    private void resetLog() {
        log = new StringBuilder();
    }

    @Override
    public MotionState state() {
        return fusionLocalizer.state();
    }

    public void update() {
        resetLog();

        fusionLocalizer.update();
        Pose pose = fusionLocalizer.pose();
        Velocity vel = fusionLocalizer.velocity();

        addLog("position", pose);
        addLog("velocity", vel);
        addLog("otos pose", relativeLocalizer.pose());
        if (tm != null) tm.drawRobot(relativeLocalizer.pose(), false);
        addLog("invalid pose", invalidPose);

        if (limelight.limelight == null) return;

        switchLimelightPipeline();
        limelight.limelight.updateRobotOrientation(toDegrees(pose.heading()) + 90);
        LLResult result = limelight.getLatestResult();

        addLog("timestamp", result != null ? result.getControlHubTimeStampNanos() : 0);
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
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                Pose3D botpose3d = invalidPose ? fiducial.getRobotPoseFieldSpace() :
                        LLUtils.getRobotPoseFieldSpace_MT2(result, fiducial);
                if (botpose3d == null) continue;
                double heading = fiducial.getRobotPoseFieldSpace().getOrientation().getYaw(RADIANS);
                Pose botpose = pose3DToPose(botpose3d).withHeading(heading);
                double tagDist = pose3DToPose(fiducial.getTargetPoseCameraSpace())
                        .distance(Pose.zero());
                double distFromCurrentEstimate = botpose.distance(pose);
                if (tagMatchesFMap(fiducial) && distFromCurrentEstimate <= MAX_DIST &&
                        tagDist <= MAX_TAG_DISTANCE &&
                        LLUtils.getAmbiguity(result, fiducial) <= MAX_AMBIGUITY) {
                    double weight = clamp(lerp((distFromCurrentEstimate - LERP_START) /
                            (MAX_DIST - LERP_START), 1, LERP_END_WEIGHT), LERP_END_WEIGHT, 1);
                    weightedTotal = weightedTotal.plus(botpose.times(weight));
                    if (tm != null) totalPose = totalPose.plus(botpose);
                    totalWeight += weight;
                    tagsCounted++;
                }
            }
            if (tagsCounted > 0) {
                weightedTotal = weightedTotal.div(totalWeight);
                LLPose = weightedTotal;
                Pose avgPose = totalPose.div(tagsCounted);
                addLog("avg dist", pose.distance(avgPose));
                addLog("weighted dist", pose.distance(LLPose));
            }
        }

        int numTags = useAll ? result.getBotposeTagCount() : tagsCounted;
        addLog("valid tags", numTags);
        addLog("total weight", totalWeight);

        if (LLPose == null) return;
        addLog("ll pose", LLPose);
        if (tm != null) tm.drawRobot(LLPose, false);

        numTags = clamp(numTags, 0, MAX_TAG_VAR_REDUCTION);
        if (numTags == 0) return;
        double linVar = useAll ? MAX_MT2_LIN_VAR / numTags : MAX_FILTERED_LIN_VAR / numTags;
        double angVar = MAX_LL_ANG_VARIANCE / numTags;
        double[] stddevs = result.getStddevMt2();
        double xVar = max(1, pow(stddevs[0] * 39.37, 2));
        double yVar = max(1, pow(stddevs[1] * 39.37, 2));
        double hVar = max(pow(toRadians(10), 2), pow(toRadians(result.getStddevMt1()[5]), 2));

        addLog("lin var", linVar);
        addLog("ang var", pow(toDegrees(sqrt(angVar)), 2));
        addLog("x var", xVar);
        addLog("y var", yVar);
        addLog("head var", pow(result.getStddevMt1()[5], 2));

        if (invalidPose) {
            setPose(LLPose);
            return;
        }

        Pose measurementVar = new Pose(xVar, yVar, hVar);
        if (result.getControlHubTimeStampNanos() != lastMeasurmentTime)
            fusionLocalizer.addMeasurement(LLPose, result.getControlHubTimeStampNanos(),
                    USE_MEASUREMENT_VARIANCE ? measurementVar : new Pose(linVar, linVar, angVar));
        lastMeasurmentTime = result.getControlHubTimeStampNanos();
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
