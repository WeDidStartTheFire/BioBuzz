package org.firstinspires.ftc.teamcode.autos.primary;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.autos.BaseAuto2;
import org.firstinspires.ftc.teamcode.autos.paths.Paths_8Pt;

public abstract class Auto_8Pt extends BaseAuto2<Paths_8Pt> {
    public static final String baseName = "8 Points";
    public static final String group = "A";
    protected final Class<Paths_8Pt> autoPathsClass = Paths_8Pt.class;

    @Override
    protected Command getAutoRoutine() {
        return sequential(
                follow(follower, paths.path1())
        );
    }
}
