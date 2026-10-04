package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OTOSConfig;
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftBack");
        c.backRightName.set("rightBack");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static OTOSConfig localizerConfig = new OTOSConfig(c -> {
        c.name.set("sensorOtos");
        c.linearScalar.set(1.01324767265);
        c.angularScalar.set(0.9756097560975604);
        c.offset.set(new Pose(5.7, 0, Math.toRadians(-90)));
        c.linearUnit.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.18378626208926785);
                Controller secondaryTranslationalForward = Controller.proportional(0.06790411298261684);
                Controller primaryTranslationalLateral = Controller.proportional(0.1800413161299527);
                Controller secondaryTranslationalLateral = Controller.proportional(0.0665204555174489);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.02077405198283903));
                c.brake.set(Controller.proportionalFeedforward(0.017657944185413176));

                c.headingFeedback.set(Controller.proportional(2.1001039050653314));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.054779787952772115, 0.0046804530273290325));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07710939084807825, 0.0667433615767754));
                c.quadraticBrakeCoefficients.set(Matrix.diag(8.597276479789705E-4, 9.170921858887551E-4));

                c.maxAchievableForwardVelocity.set(56.292125536331696);
                c.maxAchievableStrafeVelocity.set(41.18776912163885);
                c.naturalForwardDeceleration.set(36.42602748648618);
                c.naturalStrafeDeceleration.set(62.35477306926465);
            }
    );

    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return new Follower(
                new OTOSLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    public static Follower createKalmanFollower(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }

    public static Follower createLimelightFollower(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }

    public static Follower createRedundantFollower(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }
}