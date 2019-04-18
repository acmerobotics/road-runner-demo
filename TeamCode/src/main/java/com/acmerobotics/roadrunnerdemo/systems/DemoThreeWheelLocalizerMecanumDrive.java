package com.acmerobotics.roadrunnerdemo.systems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.drive.ThreeTrackingWheelLocalizer;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

@Config
public class DemoThreeWheelLocalizerMecanumDrive extends MecanumDrive {

    public static double TRACK_WIDTH = 10; //change this

    public static final List<Vector2d> TRACKING_POSITIONS = Arrays.asList(
            new Vector2d(0,0),
            new Vector2d(0,0),
            new Vector2d(0,0)
    );

    public static final List<Double> TRACKING_ORIENTATIONS = Arrays.asList(
            0.0,
            0.0,
            0.0
    );

    private LynxEmbeddedIMU imu;
    private DcMotorEx m0, m1, m2, m3;
    private DcMotor t0, t1, t2;

    public DemoThreeWheelLocalizerMecanumDrive (HardwareMap map) {
        super(TRACK_WIDTH);

        //initialize all ur hardware

        setLocalizer(new ThreeTrackingWheelLocalizer(TRACKING_POSITIONS, TRACKING_ORIENTATIONS) {
            @NotNull
            @Override
            public List<Double> getWheelPositions() {
                return Arrays.asList(
                        trackingTicksToPosition(t0.getCurrentPosition()),
                        trackingTicksToPosition(t1.getCurrentPosition()),
                        trackingTicksToPosition(t2.getCurrentPosition()));
            }
        });
    }

    @Override
    public double getExternalHeading() {
        return imu.getAngularOrientation().firstAngle; //or whatever your orientation is you can just copy this from existing code
    }

    private double ticksToPosition (double ticks) {
        return ticks; //you should convert ticks to inches here
    }

    private double trackingTicksToPosition (double ticks) {
        return ticks; // you should do some more conversion here
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                ticksToPosition(m0.getCurrentPosition()),
                ticksToPosition(m1.getCurrentPosition()),
                ticksToPosition(m2.getCurrentPosition()),
                ticksToPosition(m3.getCurrentPosition())
        );
    }

    @Override
    public void setMotorPowers(double v0, double v1, double v2, double v3) {
        m0.setPower(v0);
        m1.setPower(v1);
        m2.setPower(v2);
        m3.setPower(v3);
    }
}
