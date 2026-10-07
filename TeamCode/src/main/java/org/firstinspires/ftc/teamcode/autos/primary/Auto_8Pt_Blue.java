package org.firstinspires.ftc.teamcode.autos.primary;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.enums.Color;

@Autonomous(name = "🟦Blue🟦 " + Auto_8Pt.baseName, group = Auto_8Pt.group)
public class Auto_8Pt_Blue extends Auto_8Pt {
    @Override
    protected void configure() {
        super.configure(autoPathsClass, Color.BLUE, baseName);
    }
}
