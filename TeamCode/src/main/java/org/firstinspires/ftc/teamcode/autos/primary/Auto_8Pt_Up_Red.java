package org.firstinspires.ftc.teamcode.autos.primary;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.enums.Color;

@Autonomous(name = "🟥Red🟥 " + Auto_8Pt_Up.baseName, group = Auto_8Pt_Up.group)
public class Auto_8Pt_Up_Red extends Auto_8Pt_Up {
    @Override
    protected void configure() {
        super.configure(autoPathsClass, Color.RED, baseName);
    }
}
