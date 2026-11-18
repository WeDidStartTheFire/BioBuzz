package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static org.firstinspires.ftc.teamcode.enums.Hardware.LIMELIGHT;

import androidx.annotation.Nullable;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.enums.Hive;
import org.firstinspires.ftc.teamcode.enums.HivePosition;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;

import java.util.List;

@Configurable
public class Limelight implements Subsystem {

    public final @Nullable Limelight3A limelight;
    public static double HIVE_UP_MIN_HEIGHT = -1.20;
    public static double HIVE_DOWN_MAX_HEIGHT = -0.91;

    public Limelight(HardwareMap hardwareMap, @Nullable TelemetryUtils tm) {
        limelight = HardwareInitializer.init(hardwareMap, tm, LIMELIGHT);
        if (limelight != null) limelight.pipelineSwitch(0);
    }

    /**
     * Starts or resumes periodic polling of Limelight data.
     */
    public void start() {
        if (limelight != null) limelight.start();
    }

    @Override
    public void readSensors() {

    }

    @Override
    public void think() {

    }

    @Override
    public void update() {
    }

    /**
     * Stops polling of Limelight data.
     */
    @Override
    public void stop() {
        if (limelight != null) limelight.stop();
    }

    /**
     * @return The fiducial readings from the limelight
     */
    public @Nullable List<LLResultTypes.FiducialResult> getFiducials() {
        LLResult result = getLatestResult();
        if (result == null || !result.isValid()) return null;
        return result.getFiducialResults();
    }

    public @Nullable LLResult getLatestResult() {
        if (limelight == null) return null;
        return limelight.getLatestResult();
    }

    public Double getAverageHiveY(Hive hive) {
        double total = 0;
        int number = 0;

        if (limelight != null && getFiducials() != null) {
            for (LLResultTypes.FiducialResult fiducial : getFiducials()) {
                if (Hive.ifTagMatchesHive(fiducial.getFiducialId(), hive)) {
                    number++;
                    total += fiducial.getTargetPoseRobotSpace().getPosition().y;
                }
            }
        }

        if (number == 0) return Double.NaN;

        return total / number;
    }

    public HivePosition getHivePosition(Color alliance) {
        if (alliance == Color.BLUE) {
            HivePosition positionAccordingToAudience = HivePosition.TRANSITIONING;
            HivePosition positionAccordingToStage = HivePosition.TRANSITIONING;
            if (getAverageHiveY(Hive.BLUE_AUDIENCE) <= HIVE_UP_MIN_HEIGHT)
                positionAccordingToAudience = HivePosition.AUDIENCE;
            else if (getAverageHiveY(Hive.BLUE_AUDIENCE) >= HIVE_DOWN_MAX_HEIGHT)
                positionAccordingToAudience = HivePosition.STAGE;
            else if (Double.isNaN(getAverageHiveY(Hive.BLUE_AUDIENCE)))
                positionAccordingToAudience = HivePosition.UNKNOWN;

            if (getAverageHiveY(Hive.BLUE_STAGE) <= HIVE_UP_MIN_HEIGHT)
                positionAccordingToStage = HivePosition.STAGE;
            else if (getAverageHiveY(Hive.BLUE_STAGE) >= HIVE_DOWN_MAX_HEIGHT)
                positionAccordingToStage = HivePosition.AUDIENCE;
            else if (Double.isNaN(getAverageHiveY(Hive.BLUE_STAGE)))
                positionAccordingToStage = HivePosition.UNKNOWN;

            if (positionAccordingToAudience == positionAccordingToStage)
                return positionAccordingToAudience;
            if (positionAccordingToAudience == HivePosition.UNKNOWN)
                return positionAccordingToStage;
            if (positionAccordingToStage == HivePosition.UNKNOWN)
                return positionAccordingToAudience;
            return HivePosition.TRANSITIONING;
        }

        if (alliance == Color.RED) {
            HivePosition positionAccordingToAudience = HivePosition.TRANSITIONING;
            HivePosition positionAccordingToStage = HivePosition.TRANSITIONING;
            if (getAverageHiveY(Hive.RED_AUDIENCE) <= HIVE_UP_MIN_HEIGHT)
                positionAccordingToAudience = HivePosition.AUDIENCE;
            else if (getAverageHiveY(Hive.RED_AUDIENCE) >= HIVE_DOWN_MAX_HEIGHT)
                positionAccordingToAudience = HivePosition.STAGE;
            else if (Double.isNaN(getAverageHiveY(Hive.RED_AUDIENCE)))
                positionAccordingToAudience = HivePosition.UNKNOWN;

            if (getAverageHiveY(Hive.RED_STAGE) <= HIVE_UP_MIN_HEIGHT)
                positionAccordingToStage = HivePosition.STAGE;
            else if (getAverageHiveY(Hive.RED_STAGE) >= HIVE_DOWN_MAX_HEIGHT)
                positionAccordingToStage = HivePosition.AUDIENCE;
            else if (Double.isNaN(getAverageHiveY(Hive.RED_STAGE)))
                positionAccordingToStage = HivePosition.UNKNOWN;

            if (positionAccordingToAudience == positionAccordingToStage)
                return positionAccordingToAudience;
            if (positionAccordingToAudience == HivePosition.UNKNOWN)
                return positionAccordingToStage;
            if (positionAccordingToStage == HivePosition.UNKNOWN)
                return positionAccordingToAudience;
            return HivePosition.TRANSITIONING;
        }

        return HivePosition.UNKNOWN;
    }
}
