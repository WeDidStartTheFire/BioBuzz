package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.enums.Color;

public class Paths_Blue extends AutoPaths {
    private Pose start, startToShoot1, shoot1ToPickupStart, shoot1ToPickup, shoot1ToPickupControl1,
            pickupToShoot2Start, pickupToShoot2, shootToFinishStart, shootToFinish,
            shootToFinishControl1;

    public Paths_Blue(Color pathsColor, Color autoColor) {
        super(pathsColor, autoColor);
    }

    @Override
    protected void buildPoints() {

        start = poseFactory.of(106.4041, 130.2426, 90);
        startToShoot1 = poseFactory.of(83.75, 106.75, 270);
        shoot1ToPickupStart = poseFactory.of(83.75, 106.75, 270);
        shoot1ToPickup = poseFactory.of(132.75, 94.75, 0);
        shoot1ToPickupControl1 = poseFactory.of(117.75, 105.75, 0);
        pickupToShoot2Start = poseFactory.of(132.75, 94.75, 0);
        pickupToShoot2 = poseFactory.of(83.75, 110.75, 270);
        shootToFinishStart = poseFactory.of(83.75, 110.75, 270);
        shootToFinish = poseFactory.of(131.75, 35.75, 0);
        shootToFinishControl1 = poseFactory.of(133.75, 95.75, 0);
    }

    @Override
    public Pose start() {
        return start;
    }

    public Path startToShoot1() {
        return Paths.line(start, startToShoot1).constant(startToShoot1);
    }

    public Path shoot1ToPickup() {
        return Paths.curve(shoot1ToPickupStart, shoot1ToPickupControl1, shoot1ToPickup).linear(shoot1ToPickupStart, shoot1ToPickup);
    }

    public Path pickupToShoot2() {
        return Paths.line(pickupToShoot2Start, pickupToShoot2).linear(pickupToShoot2Start, pickupToShoot2);
    }

    public Path shootToFinish() {
        return Paths.curve(shootToFinishStart, shootToFinishControl1, shootToFinish).linear(shootToFinishStart, shootToFinish);
    }
}
