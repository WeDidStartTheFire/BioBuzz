package org.firstinspires.ftc.teamcode.autos.primary;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.enums.Color;

@Autonomous(name = "🟥Red🟥 " + Auto_8Pt_Down.baseName, group = Auto_8Pt_Down.group)
public class Auto_8Pt_Down_Red extends Auto_8Pt_Down {
    @Override
    protected void configure() {
        super.configure(autoPathsClass, Color.RED, baseName);
    }
}
