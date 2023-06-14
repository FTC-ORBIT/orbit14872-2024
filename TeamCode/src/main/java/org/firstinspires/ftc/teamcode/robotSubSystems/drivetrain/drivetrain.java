package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptTelemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;

/**
 * {@link ConceptTelemetry} illustrates various ways in which telemetry can be
 * transmitted from the robot controller to the driver station. The sample illustrates
 * numeric and text data, formatted output, and optimized evaluation of expensive-to-acquire
 * information. The telemetry {@link Telemetry#log() log} is illustrated by scrolling a poem
 * to the driver station.
 *
 * @see Telemetry
 */
public class drivetrain {
    public static final DcMotor[] motors = new DcMotor[4];
    public static BNO055IMU imu;
    private static float offSet = 0;
    private OpMode opMode;


    double[] motorsLast = new double[4];
    double motory =0;

    ElapsedTime timer = new ElapsedTime();

    private double distanceFactor = 8.5;
    private double distanceFactorXydrive = 8.5;

    private static double speedFactor = 1.0;

    public static void init(HardwareMap hardwareMap) {
        final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        motors[0] = hardwareMap.get(DcMotor.class, "lb");
        motors[1] = hardwareMap.get(DcMotor.class, "rb");
        motors[2] = hardwareMap.get(DcMotor.class, "rf");
        motors[3] = hardwareMap.get(DcMotor.class, "lf");
        for (final DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public static float wrapAnglePlusMinus180(final float angle) {
        final float wrapped = angle % 360;

        if (wrapped > 180) {
            return wrapped - 360;
        } else if (wrapped < -180) {
            return wrapped + 360;
        } else {
            return wrapped;
        }
    }

    public static float getAngle() {
        return wrapAnglePlusMinus180(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - offSet);
    }

    public double toTicks(double cm){
        return (cm / (96*3.1415*2) * 751.8) * distanceFactor;
    }
    public double toTicksXYdrive(double cm){
        return (cm / (96*3.1415*2) * 751.8) * distanceFactorXydrive;
    }

    public double speed(double minSpeed, double maxSpeed,double wantedDistance,double currentDistance) {
        double accelaration = (maxSpeed - minSpeed) / wantedDistance / 100 * 25 ;
        double dec =  (minSpeed - maxSpeed) / wantedDistance / 100 * 75;
        if (Math.abs(wantedDistance) - Math.abs(currentDistance) <= wantedDistance / 100 * 20) {
            return accelaration * currentDistance + minSpeed;
        } else if(Math.abs(wantedDistance) - Math.abs(currentDistance) >= wantedDistance/100*25 &&Math.abs(wantedDistance) - Math.abs(currentDistance) <=  wantedDistance/100*75)
            return maxSpeed;
        else
            return (dec * currentDistance + maxSpeed);
    }

    public double toCm(double ticks) {
        return ticks/751.8 * (96*3.1415);
    }

    public void resetAngle() {
        offSet = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public static void fieldCentric(final float y,final float x,final double r) {
        final Vector joystickVector = new Vector(x, y);
        final float robotAngle = (float) Math.toRadians(-getAngle());
        final Vector rotated = joystickVector.rotate(robotAngle);
        drive(rotated.y, rotated.x, r);
    }

    public void motortest() {
        while (true) {
            opMode.telemetry.addData("lb", motors[0].getCurrentPosition());
            opMode.telemetry.addData("rb", motors[1].getCurrentPosition());
            opMode.telemetry.addData("rf", motors[2].getCurrentPosition());
            opMode.telemetry.addData("lf", motors[3].getCurrentPosition());
            opMode.telemetry.update();
        }
    }
    public static void drive(double y, double x, double r) {
        final double lfPower = y + x + r;
        final double rfPower = y + x - r;
        final double lbPower = y - x - r;
        final double rbPower = y - x + r;
        motors[0].setPower(lfPower);
        motors[1].setPower(lbPower);
        motors[2].setPower(rfPower);
        motors[3].setPower(rbPower);
    }

    public static void test(){
        motors[0].setPower(0.5);
        motors[1].setPower(0.5);
        motors[2].setPower(0.5);
        motors[3].setPower(0.5);
    }
}