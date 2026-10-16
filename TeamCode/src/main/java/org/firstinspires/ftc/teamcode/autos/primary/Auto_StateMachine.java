package org.firstinspires.ftc.teamcode.autos.primary;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.autos.BaseAuto2;
import org.firstinspires.ftc.teamcode.autos.paths.Paths_StateMachine;
import org.firstinspires.ftc.teamcode.enums.Color;

public class Auto_StateMachine extends BaseAuto2<Paths_StateMachine> {
    public static final String baseName = "State Machine";
    public static final String group = "A";
    protected final Class<Paths_StateMachine> autoPathsClass = Paths_StateMachine.class;

    @Override
    protected Command getAutoRoutine() {
        return sequential(
                follow(follower, paths.path1())
        );
    }

    @Override
    protected void configure() {
        super.configure(autoPathsClass, Color.RED, baseName);
    }

}
