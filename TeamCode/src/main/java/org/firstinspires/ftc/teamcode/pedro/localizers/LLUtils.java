package org.firstinspires.ftc.teamcode.pedro.localizers;

import androidx.annotation.Nullable;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

public class LLUtils {

    private static JSONObject getMatchingFiducialJSON(LLResult result, FiducialResult fiducial) {
        String jsonString = result.toString();
        try {
            JSONObject jsonData = new JSONObject(jsonString);
            JSONArray fiducialArray = jsonData.optJSONArray("Fiducial");
            if (fiducialArray == null) return new JSONObject();
            for (int i = 0; i < fiducialArray.length(); i++) {
                JSONObject fiducialData = fiducialArray.getJSONObject(i);
                int id = fiducialData.optInt("fID", -1);
                if (id == fiducial.getFiducialId()) return fiducialData;
            }
        } catch (JSONException e) {
            return new JSONObject();
        }
        return new JSONObject();
    }

    public static double getAmbiguity(LLResult result, FiducialResult fiducial) {
        JSONObject data = getMatchingFiducialJSON(result, fiducial);
        return data.optDouble("ambig", Double.NaN);
    }

    @Nullable
    public static Pose3D getRobotPoseFieldSpace_MT2(LLResult result, FiducialResult fiducial) {
        JSONObject data = getMatchingFiducialJSON(result, fiducial);
        JSONArray jsonArray = data.optJSONArray("t6r_fs_orb");
        if (jsonArray == null || jsonArray.length() == 0) return null;
        return createPose3DRobot(parsePoseArray(jsonArray));
    }

    /**
     * Helper method to create an Pose3D instance from a pose array.
     *
     * @param pose The pose array (x, y, z, roll, pitch, yaw)
     * @return An Pose3D instance without acquisition time.
     */
    @Nullable
    private static Pose3D createPose3DRobot(double[] pose) {
        if (pose.length < 6) return null;
        Position position = new Position(DistanceUnit.METER, pose[0], pose[1], pose[2], 0);
        YawPitchRollAngles orientation = new YawPitchRollAngles(AngleUnit.DEGREES, pose[5], pose[4],
                pose[3], 0);
        return new Pose3D(position, orientation);
    }

    /**
     * Parses a JSONArray into a double array representing a robot pose. The array should contain 6
     * elements: x, y, z, roll, pitch, yaw.
     *
     * @param array The JSONArray to convert.
     * @return A double array representing the robot pose. If the input is null or has fewer than 6
     * elements, returns an array of 6 zeros.
     */
    private static double[] parsePoseArray(JSONArray array) {
        double[] pose = new double[6];  // Initialize with zeros
        if (array != null && array.length() >= 6)
            for (int i = 0; i < 6; i++) pose[i] = array.optDouble(i, 0.0);
        return pose;
    }
}
