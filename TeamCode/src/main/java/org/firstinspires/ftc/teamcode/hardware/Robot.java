package org.firstinspires.ftc.teamcode.hardware;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

import java.util.function.BooleanSupplier;


public class Robot {


    // Declare Constants
    //public static final double CORE_HEX_EDGES_PER_RADIAN = 288d / (2 * Math.PI);

    // Declare Actuators
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    public Telemetry telemetry = null;

    // Class to represent mecanum motor speed
    public class MecanumMotorSpeed {
        double[] speeds;
        double frontLeft, frontRight, backLeft, backRight;
        MecanumMotorSpeed(double frontLeft, double frontRight, double backLeft, double backRight) {
            // Load Speeds
            speeds = new double[]{frontLeft, frontRight, backLeft, backRight};
            // Find Largest Speed
            double max = Math.abs(speeds[0]);
            for(double speed : speeds) if(Math.abs(speed) > max) max = Math.abs(speed);
            // Reduce All Speeds If Max Speed Is Outside Allowed Range of [-1,1]
            if(max > 1d) for(int i = 0 ; i < speeds.length ; ++i) speeds[i] /= max;
            this.frontLeft = speeds[0];
            this.frontRight = speeds[1];
            this.backLeft = speeds[2];
            this.backRight = speeds[3];
        }
    }

    // Initialize the Hardware
    // THIS FUNCTION MUST BE CALLED
    public void init(HardwareMap hardwareMap, Telemetry tm) {

        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");



        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);


        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
//        telemetry = new TelemetryImpl(this)
        telemetry = tm;

    }

    public void vector(double drive, double strafe, double twist) {

        MecanumMotorSpeed speed = new MecanumMotorSpeed(
                (drive + strafe + twist), // front left
                (drive - strafe - twist), // front right
                (drive - strafe + twist), // back left
                (drive + strafe - twist) // back right
        );

        telemetry.addData("FrontLeft: ", "(%.2f)", speed.frontLeft);
        telemetry.addData("FrontRight: ", "(%.2f)", speed.frontRight);
        telemetry.addData("backLeft: ", "(%.2f)", speed.backLeft);
        telemetry.addData("backRight: ", "(%.2f)", speed.backRight);
        // apply calculated speeds to the drive motors
        frontLeft.setPower(speed.frontLeft);
        frontRight.setPower(speed.frontRight);
        backLeft.setPower(speed.backLeft);
        backRight.setPower(speed.backRight);

    }


}