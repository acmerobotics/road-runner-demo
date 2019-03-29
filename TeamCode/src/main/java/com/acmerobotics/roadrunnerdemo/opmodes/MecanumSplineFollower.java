package com.acmerobotics.roadrunnerdemo.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.followers.MecanumPIDVAFollower;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunnerdemo.systems.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="MecanumSplineFollower")
public class MecanumSplineFollower extends LinearOpMode {

    @Override
    public void runOpMode () {
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        MecanumPIDVAFollower follower = new MecanumPIDVAFollower(
                drive,
                MecanumDrive.TRANSLATIONAL_COEFFICIENTS,
                MecanumDrive.HEADING_COEFFICIENTS,
                0, 0, 0
        );

        Trajectory trajectory = new TrajectoryBuilder(new Pose2d(), MecanumDrive.CONSTRAINTS)
                .splineTo(new Pose2d(48, 0, Math.PI / 2), new LinearInterpolator(0, Math.PI / 2))
                .splineTo(new Pose2d(), new LinearInterpolator(Math.PI / 2, Math.PI))
                .build();

        waitForStart();

        follower.followTrajectory(trajectory);

        while (opModeIsActive() && follower.isFollowing()) {
            follower.update(drive.getPoseEstimate());
        }

    }
}
