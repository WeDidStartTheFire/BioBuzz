package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static org.firstinspires.ftc.teamcode.enums.Hardware.LIMELIGHT;

import androidx.annotation.Nullable;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.enums.Hive;
import org.firstinspires.ftc.teamcode.enums.HivePosition;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;

import java.util.List;

public class Limelight {

    private final @Nullable Limelight3A limelight;

    public Limelight(HardwareMap hardwareMap, TelemetryUtils tm) {
        limelight = HardwareInitializer.init(hardwareMap, tm, LIMELIGHT);
        if (limelight != null) limelight.pipelineSwitch(0);
    }

    /**
     * Starts or resumes periodic polling of Limelight data.
     */
    public void start() {
        if (limelight != null) limelight.start();
    }

    /**
     * Stops polling of Limelight data.
     */
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

    public double getAverageHiveY(Hive hive) {
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

        return total / number;
    }

    public HivePosition getHivePosition(Hive hive) {
        if (limelight != null && getFiducials() != null) {
            for (LLResultTypes.FiducialResult fiducial : getFiducials()) {
                if (hive == Hive.BLUE_AUDIENCE || hive == Hive.RED_AUDIENCE) {
                    if (getAverageHiveY(hive) <= -1.40 && Hive.ifTagMatchesHive(fiducial.getFiducialId(), hive))
                        return HivePosition.AUDIENCE;
                    else if (getAverageHiveY(hive) >= -1.1 && Hive.ifTagMatchesHive(fiducial.getFiducialId(), hive))
                        return HivePosition.STAGE;
                    else return HivePosition.TRANSITIONING;
                } else if (hive == Hive.BLUE_STAGE || hive == Hive.RED_STAGE) {
                    if (getAverageHiveY(hive) <= -1.40 && Hive.ifTagMatchesHive(fiducial.getFiducialId(), hive))
                        return HivePosition.STAGE;
                    else if (getAverageHiveY(hive) >= -1.1 && Hive.ifTagMatchesHive(fiducial.getFiducialId(), hive))
                        return HivePosition.AUDIENCE;
                    else return HivePosition.TRANSITIONING;
                }
            }
        }
        return HivePosition.UNKNOWN;
    }
}
