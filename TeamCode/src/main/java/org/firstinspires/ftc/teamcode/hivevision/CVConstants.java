package org.firstinspires.ftc.teamcode.hivevision;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Matrix;

@Configurable
public class CVConstants {
    public static double CAMERA_HEIGHT = 12.6;
    public static double CAMERA_PITCH = Math.toRadians(30);
    public static double FORWARD_OFFSET = 5;
    public static double LATERAL_OFFSET = 7.5;

    public static final Matrix xyzConverter = new Matrix(
            new double[][]{
                    {1, 0, 0},
                    {0, 0, 1},
                    {0, 1, 0},
            });

    public static Matrix createRotationX(double angle) {
        double c = Math.cos(angle);
        double s = Math.sin(angle);
        return new Matrix(
                new double[][]{
                        {1, 0, 0},
                        {0, c, -s},
                        {0, s, c},
                }
        );
    }

}
