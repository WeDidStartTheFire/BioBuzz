package org.firstinspires.ftc.teamcode.utilities;

import static org.firstinspires.ftc.teamcode.utils.TelemetryUtils.PrintLevel.INFO;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

@Utility(name = "Read Data", description = "Reads color sensor data from file")
@Configurable
@Disabled
public class ReadData extends OpMode {
    private TelemetryUtils tm;
    private static int i = 0;
    private File dir;
    private File file;
    private int prefixIndex = 0;
    private final String[] prefixes = {"kalman_data_", "complementary_data_", "limelight_data_",
            "colors_"};

    @Override
    public void init() {
        dir = new File(hardwareMap.appContext.getFilesDir().toURI());
        file = new File(dir, "kalman_data_0.csv");
        tm = new TelemetryUtils(telemetry);
        tm.print(file.getName(), INFO);
        tm.print(file.exists() ? "Exists" : "Does not exist", INFO);
        tm.update();
    }

    @Override
    public void init_loop() {
        boolean updateFile = false;
        if (gamepad1.aWasPressed()) {
            prefixIndex = (prefixIndex + 1) % prefixes.length;
            updateFile = true;
        }
        if (gamepad1.dpadUpWasPressed()) {
            i++;
            updateFile = true;
        }
        if (gamepad1.dpadDownWasPressed()) {
            i--;
            updateFile = true;
        }
        if (updateFile) {
            file = new File(dir, prefixes[prefixIndex] + i + ".csv");
            tm.print(file.getName(), INFO);
            tm.print(file.exists() ? "Exists" : "Does not exist", INFO);
            tm.update();
        }
    }

    @Override
    public void start() {
        file = new File(dir, prefixes[prefixIndex] + i + ".csv");
        try {
            FileReader fr = new FileReader(file);
            BufferedReader br = new BufferedReader(fr);
            Object[] lines = br.lines().toArray();
            for (Object line : lines) tm.print("", line, INFO);
            br.close();
            fr.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        tm.update();
    }

    @Override
    public void loop() {
    }
}
