package org.firstinspires.ftc.teamcode.autos.primary;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.enums.Color;

@Autonomous(name = "🟦Blue🟦 " + Auto_8Pt_Down.baseName, group = Auto_8Pt_Down.group)
public class Auto_8Pt_Down_Blue extends Auto_8Pt_Down {
    @Override
    protected void configure() {
        super.configure(autoPathsClass, Color.BLUE, baseName);
    }
}
