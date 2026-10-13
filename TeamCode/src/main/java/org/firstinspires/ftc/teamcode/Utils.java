package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static java.lang.Math.toRadians;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.jetbrains.annotations.NotNull;

import java.io.BufferedWriter;

public class Utils {

    /**
     * Linear interpolation between two values.
     *
     * @param t Interpolation parameter (0.0 = a, 1.0 = b)
     * @param a Starting value
     * @param b Ending value
     * @return Interpolated value between a and b
     */
    public static double lerp(double t, double a, double b) {
        return (b - a) * t + a;
    }

    /**
     * Saves the current pos to be used in TeleOp. Clears after power off.
     *
     * @param pos Pose to save
     */
    public static void saveOdometryPosition(@NonNull Pose pos) {
        SavedVariables.savedPose = pos;
    }

    /**
     * Saves the current pos to be used in TeleOp. Clears after power off.
     *
     * @return The position the robot ended at in the last Auto
     */
    @Nullable
    public static Pose loadOdometryPosition() {
        return SavedVariables.savedPose;
    }

    public static void addLine(BufferedWriter bw, String line) {
        try {
            bw.write(line);
            bw.newLine();
        } catch (java.io.IOException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * Converts a {@link Pose3D} to a {@link Pose} and converts from the Limelight coordinate
     * system (in meters, degrees, and different rotation) to the Pedro Pathing coordinate system
     * (in inches, radians).
     *
     * @param pose3D {@link Pose3D} to convert
     * @return Converted {@link Pose}
     */
    public static Pose pose3DToPose(@NotNull Pose3D pose3D) {
        double angle = toRadians(pose3D.getOrientation().getYaw(DEGREES) - 90);
        if (angle < 0) angle += 360;
        return new Pose(pose3D.getPosition().y / 0.0254 + 72,
                -pose3D.getPosition().x / 0.0254 + 72, angle);
    }
}
