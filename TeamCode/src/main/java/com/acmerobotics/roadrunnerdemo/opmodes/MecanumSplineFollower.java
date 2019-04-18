package com.acmerobotics.roadrunnerdemo.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.followers.MecanumPIDVAFollower;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunnerdemo.systems.DemoMecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="MecanumSplineFollower")
public class MecanumSplineFollower extends LinearOpMode {

    @Override
    public void runOpMode () {
        DemoMecanumDrive drive = new DemoMecanumDrive(hardwareMap);

        TrajectoryBuilder builder = new TrajectoryBuilder(
                new Pose2d(0,0,0),
                DemoMecanumDrive.CONSTRAINTS);

        builder
                .splineTo(new Pose2d(48, 48, Math.PI / 2), new ConstantInterpolator(0))
                .splineTo(new Pose2d(0,0,Math.PI), new LinearInterpolator(Math.PI / 2, Math.PI));

        Trajectory trajectory = builder.build();

        MecanumPIDVAFollower follower = new MecanumPIDVAFollower(
                drive,
                DemoMecanumDrive.TRANSLATIONAL_COEFFICIENTS,
                DemoMecanumDrive.HEADING_COEFFICIENTS,
                DemoMecanumDrive.K_V,
                DemoMecanumDrive.K_A,
                DemoMecanumDrive.K_STATIC
        );

        waitForStart();

        follower.followTrajectory(trajectory);

        while (opModeIsActive() && follower.isFollowing()) {
            follower.update(drive.getPoseEstimate());
        }

    }
}
