package com.acmerobotics.roadrunnerdemo.systems;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.drive.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

public class MecanumDriveWithTrackingWheels extends DemoMecanumDrive {

    private static final double TRACKING_WHEEL_RADIUS = 1;
    private static final double TRACKING_ENCODER_TICKS_PER_REVOLUTION = 500;

    private static final List<Vector2d> TRACKING_WHEEL_POSITIONS = Arrays.asList(
            new Vector2d(4, 0),
            new Vector2d(0, 4)
    );

    private static final List<Double> TRACKING_WHEEL_ORIENTATIONS = Arrays.asList(
            0.0,
            Math.PI / 2
    );

    private DcMotor encoder0, encoder1;

    public MecanumDriveWithTrackingWheels (HardwareMap map) {
        super(map);

        encoder0 = map.dcMotor.get("encoder0");
        encoder1 = map.dcMotor.get("encoder1");

        setLocalizer(new TwoTrackingWheelLocalizer(TRACKING_WHEEL_POSITIONS, TRACKING_WHEEL_ORIENTATIONS) {
            @NotNull
            @Override
            public List<Double> getWheelPositions() {
                return Arrays.asList(
                        trackingWheelTicksToDistance(encoder0.getCurrentPosition()),
                        trackingWheelTicksToDistance(encoder1.getCurrentPosition())
                );
            }

            @Override
            public double getHeading() {
                return getExternalHeading();
            }
        });
    }

    private double trackingWheelTicksToDistance (int ticks) {
        double rotations = ticks / TRACKING_ENCODER_TICKS_PER_REVOLUTION;
        return rotations * 2 * Math.PI * TRACKING_WHEEL_RADIUS;
    }

}
