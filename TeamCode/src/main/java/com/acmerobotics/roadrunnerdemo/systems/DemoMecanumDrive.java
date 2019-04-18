package com.acmerobotics.roadrunnerdemo.systems;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.hardware.HardwareDeviceManager;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

public class DemoMecanumDrive extends MecanumDrive {

    public static PIDCoefficients TRANSLATIONAL_COEFFICIENTS = new PIDCoefficients(
            1,
            0,
            0
    );
    public static PIDCoefficients HEADING_COEFFICIENTS = new PIDCoefficients(
            0,
            0,
            0
    );

    public static double K_V = 0;
    public static double K_A = 0;
    public static double K_STATIC = 0;

    public static DriveConstraints CONSTRAINTS = new DriveConstraints(
            10,
            10,
            10,
            10
    );

    public static final double TRACK_WIDTH = 16;
    public static final double WHEEL_BASE = 16;
    public static final double WHEEL_RADIUS = 2;
    public static final double GEAR_RATIO = 1;

    private LynxEmbeddedIMU imu;
    private DcMotorEx m0, m1, m2, m3;

    public DemoMecanumDrive(HardwareMap map) {
        super(TRACK_WIDTH, WHEEL_BASE);

        imu = map.get(LynxEmbeddedIMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        m0 = map.get(DcMotorEx.class, "motor0");
        m1 = map.get(DcMotorEx.class, "motor1");
        m2 = map.get(DcMotorEx.class, "motor2");
        m3 = map.get(DcMotorEx.class, "motor3");

        m2.setDirection(DcMotorSimple.Direction.REVERSE);
        m3.setDirection(DcMotorSimple.Direction.REVERSE);

        List<DcMotorEx> motors = Arrays.asList(m0, m1, m2, m3);
        for (DcMotorEx motor: motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }

    @Override
    public double getExternalHeading () {
        return imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle;
    }

    private double motorTicksToWheelPosition (int ticks) {
        double rotations = ticks / (m0.getMotorType().getTicksPerRev() * GEAR_RATIO);
        return rotations * 2 * Math.PI * WHEEL_RADIUS;
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions () {
        return Arrays.asList(
                motorTicksToWheelPosition(m0.getCurrentPosition()),
                motorTicksToWheelPosition(m1.getCurrentPosition()),
                motorTicksToWheelPosition(m2.getCurrentPosition()),
                motorTicksToWheelPosition(m3.getCurrentPosition())
        );
    }

    @Override
    public void setMotorPowers (double v0, double v1, double v2, double v3) {
        m0.setPower(v0);
        m1.setPower(v1);
        m2.setPower(v2);
        m3.setPower(v3);
    }

}
