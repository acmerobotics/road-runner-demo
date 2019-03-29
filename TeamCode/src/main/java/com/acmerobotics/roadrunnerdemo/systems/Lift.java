package com.acmerobotics.roadrunnerdemo.systems;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    public static double K_V = 0;
    public static double K_A = 0;
    public static double K_STATIC = 0;

    public static double G = 0;

    public static double MAX_V = 0;
    public static double MAX_A = 0;
    public static double MAX_J = 0;

    public static PIDCoefficients coefficients = new PIDCoefficients(
            0,
            0,
            0
    );

    public static final double WINCH_RADIUS = 1;
    public static final double ENCODER_TICKS_PER_REVOLUTION = 1;

    private MotionProfile profile;
    private PIDFController controller;
    private long profileStartTime;
    private double offset;
    private double driverCommandedPower;
    private boolean driverControlled;

    private DcMotor motor;


    public Lift (HardwareMap map) {
        motor = map.dcMotor.get("lift");
        controller = new PIDFController(coefficients, K_V, K_A, K_STATIC, (x) -> G);
        setPosition(0);
        goToPosition(0);
    }

    public void update () {
        if (driverControlled) {
            motor.setPower(driverCommandedPower);
        } else {
            MotionState target = profile.get(getProfileTime());
            controller.setTargetPosition(target.getX());
            motor.setPower(controller.update(getPosition(), target.getV(), target.getA()));
        }
    }

    public void goToPosition (double targetPosition) {
        MotionState start = new MotionState(getPosition(), 0,0,0);
        if (profile != null) {
            start = profile.get(getProfileTime());
        }
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start,
                new MotionState(targetPosition, 0, 0, 0),
                MAX_V,
                MAX_A,
                MAX_J
        );
        driverControlled = false;
        profileStartTime = System.currentTimeMillis();
    }

    public double getPosition () {
        return internalGetPosition() + offset;
    }

    public void setPower (double power) {
        if (!driverControlled) controller.reset();
        driverControlled = true;
        driverCommandedPower = power;
    }

    public boolean isFollowingProfile () {
        return profile != null && getProfileTime() < profile.duration();
    }

    private void setPosition (double position) {
        offset = position - internalGetPosition();
    }

    private double internalGetPosition () {
        double rotations = motor.getCurrentPosition() / ENCODER_TICKS_PER_REVOLUTION;
        return rotations * 2 * Math.PI * WINCH_RADIUS;
    }

    private double getProfileTime () {
        return (System.currentTimeMillis() - profileStartTime) / 1000.0;
    }
}
