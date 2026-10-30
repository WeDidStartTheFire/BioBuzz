package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.enums.Color;

public class Paths_Red extends AutoPaths {
    private Pose start, startToShoot1, shootToPrepPickupStart, shootToPrepPickup,
            prepPickupToPickup, pickupToBackUp, pickupToBackUpControl1,
            backUpToShoot2Start, backUpToShoot2, shoot2ToPark1,
            shoot2ToPark1Control1, park1ToPark2, park2ToPark3;

    public Paths_Red(Color pathsColor, Color autoColor) {
        super(pathsColor, autoColor);
    }

    @Override
    protected void buildPoints() {
        start = poseFactory.of(31.1464, 8, 90);
        startToShoot1 = poseFactory.of(59, 23, 90);
        shootToPrepPickupStart = poseFactory.of(59, 23, 90);
        shootToPrepPickup = poseFactory.of(10.75, 16.75, -90);
        prepPickupToPickup = poseFactory.of(10.75, 8.75, -90);
        pickupToBackUp = poseFactory.of(28.75, 28.75, -170.5377);
        pickupToBackUpControl1 = poseFactory.of(10.75, 25.75, 0);
        backUpToShoot2Start = poseFactory.of(28.75, 28.75, 180);
        backUpToShoot2 = poseFactory.of(58.75, 22.75, 90);
        shoot2ToPark1 = poseFactory.of(16.75, 88.75, 86.4237);
        shoot2ToPark1Control1 = poseFactory.of(13.75, 40.75, 0);
        park1ToPark2 = poseFactory.of(16.75, 94.75, -90);
        park2ToPark3 = poseFactory.of(10.75, 94.75, 0);
    }

    @Override
    public Pose start() {
        return start;
    }


    public Path startToShoot1() {
        return Paths.line(start, startToShoot1).constant(startToShoot1);
    }

    public Path shootToPrepPickup() {
        return Paths.line(shootToPrepPickupStart, shootToPrepPickup).linear(shootToPrepPickupStart, shootToPrepPickup);
    }

    public Path prepPickupToPickup() {
        return Paths.line(shootToPrepPickup, prepPickupToPickup).tangent();
    }

    public Path pickupToBackUp() {
        return Paths.curve(prepPickupToPickup, pickupToBackUpControl1, pickupToBackUp).reverseTangent();
    }

    public Path backUpToShoot2() {
        return Paths.line(backUpToShoot2Start, backUpToShoot2).linear(backUpToShoot2Start, backUpToShoot2);
    }

    public Path shoot2ToPark1() {
        return Paths.curve(backUpToShoot2, shoot2ToPark1Control1, shoot2ToPark1).tangent();
    }

    public Path park1ToPark2() {
        return Paths.line(shoot2ToPark1, park1ToPark2).reverseTangent();
    }

    public Path park2ToPark3() {
        return Paths.line(park1ToPark2, park2ToPark3).reverseTangent();
    }

}
