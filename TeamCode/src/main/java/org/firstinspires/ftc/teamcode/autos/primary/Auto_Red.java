package org.firstinspires.ftc.teamcode.autos.primary;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.autos.BaseAuto2;

public class Auto_Red extends BaseAuto2<org.firstinspires.ftc.teamcode.autos.paths.Paths_Red> {
    public static final String baseName = "Red";
    public static final String group = "A";
    protected final Class<org.firstinspires.ftc.teamcode.autos.paths.Paths_Red> autoPathsClass = org.firstinspires.ftc.teamcode.autos.paths.Paths_Red.class;

    @Override
    protected Command getAutoRoutine() {
        return sequential(
                follow(follower, paths.startToShoot1()),
                follow(follower, paths.shootToPrepPickup()),
                follow(follower, paths.prepPickupToPickup()),
                follow(follower, paths.pickupToBackUp()),
                follow(follower, paths.backUpToShoot2()),
                follow(follower, paths.shoot2ToPark1()),
                follow(follower, paths.park1ToPark2()),
                follow(follower, paths.park2ToPark3())
        );
    }

    @Override
    protected void configure() {

    }


}
