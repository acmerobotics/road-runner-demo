package com.acmerobotics.roadrunnerdemo.opmodes;

import android.os.Environment;
import android.util.Log;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunnerdemo.systems.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.FileWriter;

public class DriveTrainCharcaterization extends LinearOpMode {

    public static double a = .01;

    @Override
    public void runOpMode () {
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        FileWriter writer = null;
        try {
            writer = new FileWriter(Environment.getExternalStorageDirectory().getAbsolutePath() + "ACME/drivetrain" + System.currentTimeMillis() / 10000 + ".csv");
            writer.write("command, actual\n");
        } catch (Exception e) {
            Log.e("ahh", e.getLocalizedMessage());
        }

        waitForStart();


        double lastCommand = 0;
        double lastTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            try {
                writer.write(lastCommand + ", " + drive.getVelocity() + '\n');
            } catch (Exception e) {
                Log.e("ahh", e.getLocalizedMessage());

            }
            double now = System.currentTimeMillis();
            double dt = (now - lastTime) / 1000.0;
            lastTime = now;
            lastCommand += dt * a;
            drive.setVelocity(new Pose2d(lastCommand, 0, 0));
        }


    }
}
