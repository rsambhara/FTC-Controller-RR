package org.firstinspires.ftc.teamcode.hardware;


import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.auton.Constants;
import org.firstinspires.ftc.teamcode.auton.PIDFController;
import org.firstinspires.ftc.teamcode.auton.TurnPIDController;


public class RobotElevator {

    // Declare Constants
    //public static final double CORE_HEX_EDGES_PER_RADIAN = 288d / (2 * Math.PI);


    // Declare Actuators
    private DcMotor elevator = null;
    private DcMotor claw = null;
    public Telemetry telemetry = null;
    //private IMU imu = null;
    /*public BNO055IMU imu = null;
    public IMU imuNew;



    private Orientation lastAngles = new Orientation();
    private double currentAngle = 0d; // Gives what direct is the robot facing*/
    //private static final PIDFController pidOld = new PIDFController(0.001, 0d, 0d, 0d);


    // Initialize the Hardware
    // THIS FUNCTION MUST BE CALLED
    public void init(HardwareMap hardwareMap, Telemetry tm) {

        // adding elevator and claw motors
        elevator = hardwareMap.get(DcMotor.class,"elevator");
        claw = hardwareMap.get(DcMotor.class,"claw");

        elevator.setDirection(DcMotorSimple.Direction.REVERSE);
        claw.setDirection(DcMotorSimple.Direction.REVERSE);

        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setPower(0d);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       /* // Expansion Hub IMU initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; *//* see the calibration sample opmode *//*
*/
//        parameters.loggingEnabled      = true;
       /* parameters.loggingTag          = "IMU";
        parameters.loggingEnabled      = true;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Control hub New IMU initialization
        imuNew = hardwareMap.get(IMU.class, "imuNew");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imuNew.initialize(new IMU.Parameters(orientationOnRobot));
        imuNew.resetYaw();*/


        // IMU Initialization end

        telemetry = tm;

    }



    public double getElevatorPosition() {
        return elevator.getCurrentPosition();
    }

    public void setElevatorPower(double power){
        elevator.setPower(power);
    }
    public void setClawPower(double cPower){
        claw.setPower(cPower);
    }


}