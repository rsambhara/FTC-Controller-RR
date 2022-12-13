package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp(name="Driver Controlled")
@Disabled
public class MainTeleOpMode extends OpMode {

    // Declare Hardware object
    Robot robot;

    // Constructor
    public MainTeleOpMode() {

        // Instantiate Objects
        robot = new Robot();

    }

    @Override
    public void init() {

        // Initialize Hardware
        robot.init(hardwareMap, telemetry);

    }

    @Override
    public void loop() {

        /* Drivetrain */
        // read inputs from gamepad
        double Y1 = Math.pow(gamepad1.left_stick_y, 3);
        double Y2 = Math.pow(gamepad1.right_stick_y, 3);
        double X = Math.pow(gamepad1.right_stick_x, 3);
        // calculate drive vector from gamepad input
        // forward is the average of the left and right sticks in the y direction
        // range is from -1.0 to +1.0
        double forward = -(Y1+Y2)/2;
        // right, aka strafe, is simply the x value from the right stick
        // range is from -1.0 to +1.0
        double right = X;
        // clockwise, aka twist, is the difference between the left and right sticks in the y direction
        // range is from -1.0 to +1.0
        double clockwise = -(Y1-Y2)/2;
        // Add telemetry for the raw inputs
        telemetry.addData("Drive: ", "(%.2f)", forward);
        telemetry.addData("Strafe: ", "(%.2f)", right);
        telemetry.addData("Twist: ","(%.2f)",clockwise);

        // Set a scale factor to reduce the sensitivity of the forward and twist motions
        double forwardscalefactor = 0.8;
        double rightscalefactor = 1.0;
        double clockwisescalefactor = 0.5;
        // Send drive vector to the robot object
        robot.vector(forward*forwardscalefactor, right*rightscalefactor, clockwise*clockwisescalefactor);

    }

}