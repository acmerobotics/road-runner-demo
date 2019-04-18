package com.acmerobotics.roadrunnerdemo.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunnerdemo.systems.Lift;
import com.acmerobotics.roadrunnerdemo.systems.DemoMecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="teleop")
public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode () {
        Drive drive = new DemoMecanumDrive(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                lift.goToPosition(10);
            } else if (gamepad1.dpad_down) {
                lift.goToPosition(0);
            }

            lift.update();

            drive.setVelocity(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

        }
    }

}
