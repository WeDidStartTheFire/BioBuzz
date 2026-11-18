package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.enums.Color;

public class Paths_Red extends AutoPaths {
    private Pose start, startToShoot1, shoot1ToPickupStart, shoot1ToPickup, shoot1ToPickupControl1,
            pickupToShoot2Start, pickupToShoot2, pickupToShoot2Control1, shootToFinishStart, shootToFinish,
            shootToFinishControl1, shootToFinishControl2;

    public Paths_Red(Color pathsColor, Color autoColor) {
        super(pathsColor, autoColor);
    }

    @Override
    protected void buildPoints() {
        start = poseFactory.of(56, 8, 90);
        startToShoot1 = poseFactory.of(56, 36, 90);
        shoot1ToPickupStart = poseFactory.of(56, 36, 90);
        shoot1ToPickup = poseFactory.of(8.5876, 46.8857, 180);
        shoot1ToPickupControl1 = poseFactory.of(29.283, 36.6469, 0);
        pickupToShoot2Start = poseFactory.of(8.5876, 46.8857, 180);
        pickupToShoot2 = poseFactory.of(55.75, 31.75, 90);
        pickupToShoot2Control1 = poseFactory.of(26.75, 29.75, 0);
        shootToFinishStart = poseFactory.of(55.75, 31.75, 90);
        shootToFinish = poseFactory.of(11.75, 105.75, 180);
        shootToFinishControl1 = poseFactory.of(11.75, 37.75, 0);
        shootToFinishControl2 = poseFactory.of(21.75, 75.75, 0);
    }

    @Override
    public Pose start() {
        return start;
    }
    public Path startToShoot1() {
        return Paths.line(start, startToShoot1).constant(startToShoot1);
    }

    public Path shoot1ToPickup() {
        return Paths.curve(shoot1ToPickupStart, shoot1ToPickupControl1,
                shoot1ToPickup).linear(shoot1ToPickupStart, shoot1ToPickup);
    }

    public Path pickupToShoot2() {
        return Paths.curve(pickupToShoot2Start, pickupToShoot2Control1,
                pickupToShoot2).linear(pickupToShoot2Start, pickupToShoot2);
    }

    public Path shootToFinish() {
        return Paths.curve(shootToFinishStart, shootToFinishControl1, shootToFinishControl2,
                shootToFinish).linear(shootToFinishStart, shootToFinish);
    }

}
