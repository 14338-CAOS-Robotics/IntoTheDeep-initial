package org.firstinspires.ftc.teamcode.Hardware.Physical;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Color_Sensor;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Distance_Sensor;

//hardware variables followed by hardware objects

//DCMotorEx: https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html?com/qualcomm/robotcore/hardware/DcMotorEx.html
@Config
public class CAOSHardware extends Hardware {

    public VoltageSensor batteryVoltage;


    public static final double HORIZONTAL_TICKS_PER_CM = -0.00132968; // -0.001927 everything *243.84/126.5;
    public static final double VERTICAL_TICKS_PER_CM = 0.00132968; //should be different than one another (*60/48);

    public BNO055IMU imu;

    public DcMotor intakeExtend, activeIntake, lift;

    //NOTE: THESE WILL BE DECLARED IN THE INDIVIDUAL SUBSYSTEMS. HERE TO KEEP TRACK OF
    public Servo claw, intakePitchLeft, intakePitchRight, outtakePitchLeft, outtakePitchRight, outtakeRoll;

    private HardwareMap hardwareMap;

    private double imuOffset = 0;
    private int imuNumber = 0;
    Distance_Sensor leftIntakeDist, rightIntakeDist;
    Color_Sensor intakeSensor, outtakeSensor;


    @Override
    public double getLeftBackPowerScalar() {
        return -1;
    }

    @Override
    public double getRightBackPowerScalar() {
        return -1;
    }

    @Override
    public double getRightFrontPowerScalar() {
        return -1;
    }

    @Override
    public double getLeftFrontPowerScalar() {
        return -1;
    }

    @Override
    public double getVerticalEncoderTicksToCm() {
        return VERTICAL_TICKS_PER_CM; //should be different than one another (*60/48)
    }

    @Override
    public double getHorizontalEncoderTicksToCm() {
        return HORIZONTAL_TICKS_PER_CM; // -0.001927 everything *243.84/126.5
    }
    @Override
    public double getBatteryVoltage(){return batteryVoltage.getVoltage();}

    @Override
    public void driveWithEncoders() {
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void driveWithoutEncoders() {
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public CAOSHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();

        //DRIVE
        rightFront = hardwareMap.dcMotor.get("rf");
        leftFront = hardwareMap.dcMotor.get("lf");
        leftBack = hardwareMap.dcMotor.get("lb");
        rightBack = hardwareMap.dcMotor.get("rb");

        //OTHER
        activeIntake = hardwareMap.dcMotor.get("ia");
        intakeExtend = hardwareMap.dcMotor.get("ie");
        lift = hardwareMap.dcMotor.get("ol");

//        lClaw = hardwareMap.servo.get("leftClaw");
//        rClaw = hardwareMap.servo.get("rightClaw");
//        lClaw.setDirection(Servo.Direction.REVERSE);
//        rClaw.setDirection(Servo.Direction.FORWARD);


        //REVERSE STUFF HERE
        //DCMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        verticalEncoderLeft = rightFront;
        verticalEncoderRight = leftBack;
        horizontalEncoder = leftFront;

        horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalEncoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalEncoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        driveWithoutEncoders();

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);


//        intakeAngle = hardwareMap.servo.get("ia");
//        intakeTilt = hardwareMap.servo.get("it");
//        IntakeLeftClaw = hardwareMap.servo.get("ilc");
//        IntakeRightClaw = hardwareMap.servo.get("irc");
//        leftOuttake4b = hardwareMap.servo.get("lo4b");
//        rightOuttake4b = hardwareMap.servo.get("ro4b");
//        outtakePitch = hardwareMap.servo.get("op");
//        outtakeRoll = hardwareMap.servo.get("or");
//        leftOuttake = hardwareMap.servo.get("lo");
//        rightOuttake = hardwareMap.servo.get("ro");
//        planeLaunch =  hardwareMap.servo.get("pl");


//        leftIntakeDist = new Distance_Sensor("lid");
//        rightIntakeDist = new Distance_Sensor("rid");

    }

    @Override
    public double getImuHeading(){
        Orientation angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        double heading = (angles.firstAngle + 360 + imuOffset) % 360;
        return Math.toRadians(heading);
    }

    public void switchIMU(){
        imuOffset = Math.toDegrees(getImuHeading());
        imuNumber = (imuNumber + 1) % 2;
        switch (imuNumber){
            case 0:
                imu =  hardwareMap.get(BNO055IMU.class, "imu1");
                break;
            case 1:
                imu =  hardwareMap.get(BNO055IMU.class, "imu2");
                break;
        }
        BNO055IMU.Parameters Params = new BNO055IMU.Parameters();
        Params.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        Params.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Params.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opMode
        Params.loggingEnabled      = true;
        Params.loggingTag          = "IMU";
        Params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(Params);
    }

}
