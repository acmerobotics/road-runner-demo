package com.acmerobotics.roadrunnerdemo.opmodes;

import android.os.Environment;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunnerdemo.systems.DemoMecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.FileWriter;

@Config
@Autonomous(name="drivetrainCharacterization")
public class DriveTrainCharcaterization extends LinearOpMode {

    public static double a = .01;

    public static final String TAG = "dtCharacterization";

    @Override
    public void runOpMode () {
        DemoMecanumDrive drive = new DemoMecanumDrive(hardwareMap);

        FileWriter writer = null;
        try {
            writer = new FileWriter(Environment.getExternalStorageDirectory().getPath() + "/TUNING/drivetrain" + System.currentTimeMillis() / 10000 + ".csv");
            writer.write("command, velocity");
        } catch (Exception e) {
            Log.e(TAG, e.getLocalizedMessage());
            return;
        }

        waitForStart();

        double lastCommand = 0;
        double lastTime = System.currentTimeMillis();
        double lastPosition = 0;
        drive.setPoseEstimate(new Pose2d());

        while (opModeIsActive()) {
            double now = System.currentTimeMillis();
            double dt = (now - lastTime) / 1000.0;
            lastTime = now;
            lastCommand += dt * a;
            double position = drive.getPoseEstimate().getX();
            double velocity = (position - lastPosition) / dt;
            lastPosition = position;

            try {
                writer.write(lastCommand + ", " + velocity + '\n');
            } catch (Exception e) {
                Log.e(TAG, e.getLocalizedMessage());
                return;
            }

            drive.setVelocity(new Pose2d(lastCommand, 0, 0));
        }

        try {
            writer.flush();
            writer.close();
        } catch (Exception e) {
            Log.e(TAG, e.getLocalizedMessage());
        }


    }
}
