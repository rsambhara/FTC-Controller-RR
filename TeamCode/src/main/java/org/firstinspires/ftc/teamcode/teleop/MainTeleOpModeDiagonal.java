package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.auton.Constants;
import org.firstinspires.ftc.teamcode.auton.PIDFController;
import org.firstinspires.ftc.teamcode.auton.PIDFMotorController;
import org.firstinspires.ftc.teamcode.auton.TurnPIDController;
import org.firstinspires.ftc.teamcode.hardware.RobotDiagonal;

import java.text.DecimalFormat;

@TeleOp(name="Driver Controlled Diagonal")
public class MainTeleOpModeDiagonal extends OpMode {

    // Declare Hardware object
    RobotDiagonal robot;
    private static final PIDFController pidElevator = new PIDFController(0.001, 0d, 0d, 0d);
    private static final PIDFMotorController pidMotorLeftF = new PIDFMotorController(0, 0d, 0d, 1);
    private static final PIDFMotorController pidMotorLeftB = new PIDFMotorController(0, 0d, 0d, 1);
    private static final PIDFMotorController pidMotorRightF = new PIDFMotorController(0, 0d, 0d, 1);
    private static final PIDFMotorController pidMotorRightB = new PIDFMotorController(0, 0d, 0d, 1);
    public static int setElevatorHeight = 0;
    public static final int maxElevatorHeight = 3250;
    // gives the last recorded angle of the robot
    private Orientation lastAngles = new Orientation();
    private double currentAngle = 0d; // Gives what direct is the robot facing

    // Constructor
    public MainTeleOpModeDiagonal() {

        // Instantiate Objects
        robot = new RobotDiagonal();

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

        double gp1LeftY = gamepad1.left_stick_y;
        double gp1RightY = gamepad1.right_stick_y;
        double gp1RightX = gamepad1.right_stick_x;
        boolean buttonPress = false;

        //ROBOT movements defined as
        // A = Move backward, B = Strafe right
        // X = Strafe left, Y = Move forward

/*setConstantSpeedsByButtons(gp1LeftY,gp1RightY,gp1RightX); */
        if(gamepad1.y){
            // Move forward
            gp1LeftY = - Constants.ROBOT_MOVE_STRAIGHT_Y;
            gp1RightX = 0;
            gp1RightY = changeSpeed(gamepad1.left_stick_y, gp1LeftY);
            buttonPress = true;
        } else if(gamepad1.a) {
            // Move backward
            gp1LeftY = Constants.ROBOT_MOVE_STRAIGHT_Y;
            gp1RightX = 0;
            gp1RightY = changeSpeed(gamepad1.left_stick_y, gp1LeftY);
            buttonPress = true;
        } else if(gamepad1.b) {
            // Strafe Right
            double strafe = Constants.ROBOT_STRAFE_Y;
            strafe = changeSpeed(gamepad1.left_stick_x,strafe);
            strafe = Math.abs(strafe);
            gp1LeftY = - strafe;
            gp1RightX = 0;
            gp1RightY = strafe;
            buttonPress = true;
        } else if(gamepad1.x){
            // Strafe Left
            double strafe = Constants.ROBOT_STRAFE_Y;
            strafe = changeSpeed(gamepad1.left_stick_x,strafe);
            strafe = Math.abs(strafe);
            gp1LeftY = strafe;
            gp1RightX = 0;
            gp1RightY = - strafe;
            buttonPress = true;
        } else if(gamepad1.right_bumper){
            // Twist right
            double twist = Constants.ROBOT_TURN_X;
            gp1RightX = changeSpeed(gamepad1.left_stick_x,twist);
            gp1LeftY = 0;
            gp1RightY =0;
        } else if(gamepad1.left_bumper){
            // Twist left
            double twist = - Constants.ROBOT_TURN_X;
            gp1RightX = changeSpeed(gamepad1.left_stick_x,twist);
            gp1LeftY = 0;
            gp1RightY =0;
        }

        telemetry.addLine("Gamepad 1 Left Y : " +gp1LeftY+",  Right Y : "+gp1RightY);
        telemetry.addLine("Gamepad 1 Right X : "+gp1RightX);
        // Commenting the below lines. These were added to reduce the speed of the
        // Robot.
//        double Y1 = gp1LeftY * Math.abs(gp1LeftY);
//        double Y2 = gp1RightY * Math.abs(gp1RightY);
//        double X = gp1RightX * Math.abs(gp1RightX);
        // Removing the lines
        double Y1 = gp1LeftY * 0.8 ;
        double Y2 = gp1RightY * 0.8;
        double X = gp1RightX * 0.8;
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
        //Extract values from gamepad 2

        // Here claw negative is open and positive is close
        //With the sensivity of the claw material pass the max power as 0.2
        double clawX = gamepad2.left_stick_x * 0.5;
        // Elevator
        // Negative power is to go up, positive power is to come down
        // For elevator to stay stable up it should have atleast .10 power
        // While coming down it is coming down slowly if power is 0.05
        double elevatorY = gamepad2.right_stick_y * 0.5;
        clawX = getFormattedDouble(clawX);
        elevatorY = getFormattedDouble(elevatorY);

        telemetry.addData("Claw movement, Elevator Movement ","%f : %f ",clawX,elevatorY);


        // Add telemetry for the raw inputs
//        telemetry.addData("Drive: ", "(%.2f)", forward);
//        telemetry.addData("Strafe: ", "(%.2f)", right);
//        telemetry.addData("Twist: ","(%.2f)",clockwise);
        telemetry.addData("Drive: ", "(%.2f)", forward);
        telemetry.addData("Twist: ", "(%.2f)", right);
        telemetry.addData("Strafe: ","(%.2f)",clockwise);
        // Set a scale factor to reduce the sensitivity of the forward and twist motions
        double fwdScaleFactor = 0.8;
        double rightScaleFactor = 1.0;
        double clockwiseScaleFactor = 0.8;
        if(buttonPress){
            fwdScaleFactor = 1.0;
            clockwiseScaleFactor = 1.0;
            rightScaleFactor = 1.0;
        }
        double elevatorSlidePower = 0.0;
        // Send drive vector to the robot object
        robot.vector(forward*fwdScaleFactor, right*rightScaleFactor, clockwise*clockwiseScaleFactor);
        if(gamepad2.y){
            //Highest point
            setElevatorHeight = Constants.HIGH_JUNCTION;
            setElevatorHeight = Math.min(setElevatorHeight,maxElevatorHeight);
            telemetry.addData("RD: MainMode, High Junction",setElevatorHeight);
        }
        if(gamepad2.x) {
            setElevatorHeight = Constants.MEDIUM_JUNCTION;
            setElevatorHeight = Math.min(setElevatorHeight,maxElevatorHeight);
        }
        if(gamepad2.a) {
            setElevatorHeight = Constants.LOW_JUNCTION;
            setElevatorHeight = Math.min(setElevatorHeight,maxElevatorHeight);
            telemetry.addData("RD: MainMode, Low Junction ", setElevatorHeight);
        }
        if(gamepad2.b){
            setElevatorHeight = Constants.GROUND_LEVEL;
            setElevatorHeight = Math.min(setElevatorHeight,maxElevatorHeight);
            telemetry.addData("RD: MainMode, Base positions ", setElevatorHeight);
            robot.setClawPower(-0.25);
        }
        if(gamepad2.left_bumper){
            setElevatorHeight = setElevatorHeight - 10;
            setElevatorHeight = Math.min(setElevatorHeight,maxElevatorHeight);
//            robot.changeElevatorHeightByUnits(-10);
        }
        if(gamepad2.right_bumper){
            setElevatorHeight = setElevatorHeight + 10;
            setElevatorHeight = Math.min(setElevatorHeight,maxElevatorHeight);
        }
        if( gamepad2.right_trigger > 0){
            clawX = -0.45;
        }
        if(gamepad2.left_trigger > 0){
            clawX = 0.35;
        }
        elevatorSlidePower = pidElevator.calculate(robot.getElevatorPosition(),setElevatorHeight);
        robot.setElevatorPower(elevatorSlidePower);
        robot.setClawPower(clawX);
    }

    private double getFormattedDouble(double inputValue) {
        DecimalFormat df = new DecimalFormat("#.###");
        return Double.parseDouble(df.format(inputValue));
    }

    private double changeSpeed(double changeSpeed, double endValue) {
        if(changeSpeed != 0){
            endValue = getFormattedDouble(endValue + (changeSpeed));
        }
        return endValue;
    }

    public void resetAngle(){
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentAngle = 0.0;
    }


    public double getCurrentAngle(){
        Orientation orient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = orient.firstAngle - lastAngles.firstAngle;

        if (deltaAngle > 180)  {
            deltaAngle -= 360;
        } else if (deltaAngle <= -180) {
            deltaAngle += 360;
        }
        currentAngle += deltaAngle;
        lastAngles = orient;
        return currentAngle;
    }


    public double turnAngle(boolean opMode, double angle){
        resetAngle();
        double error = angle;
        while(opMode && Math.abs(error)>2){
            double turnPower = (error < 0 ? - Constants.ANGLE_TURN_SPEED : Constants.ANGLE_TURN_SPEED);
            robot.setAllWheelPower(turnPower,-turnPower,turnPower,-turnPower);
            error = angle - getCurrentAngle();
        }
        robot.setAllWheelPower(0,0,0,0);
        return currentAngle;
    }

    public void turnAnglePID(boolean opMode, double angle){
        turnAbsolutePID(true,angle + getAbsoluteAngle());
    }

    public void turnAbsolute(boolean opMode, double angle){
        Orientation orient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double error = angle - orient.firstAngle;

        if (error > 180)  {
            error -= 360;
        } else if (error <= -180) {
            error += 360;
        }
        turnAngle(true,error);
    }


    public void turnAbsolutePID(boolean opMode, double angle){

        TurnPIDController pid = new TurnPIDController(angle, 0.01, 0, 0.003);
        // Checking lastSlope to make sure that it's not oscillating when it quits
        while (Math.abs(angle - getAbsoluteAngle()) > 0.5 || pid.getLastSlope() > 0.75) {
            double motorPower = pid.update(getAbsoluteAngle());
            robot.setAllWheelPower(-motorPower, motorPower, -motorPower, motorPower);

            telemetry.addData("Current Angle", getAbsoluteAngle());
            telemetry.addData("Target Angle", angle);
            telemetry.addData("Slope", pid.getLastSlope());
            telemetry.addData("Power", motorPower);
            telemetry.update();
        }
        robot.setAllWheelPower(0,0,0,0);

    }

    public double getAbsoluteAngle(){
        return robot.imu.getAngularOrientation
                (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    @Deprecated
    private void setConstantSpeedsByButtons(double gp1LeftY,double gp1RightY,double gp1RightX){
        //ROBOT movements defined as
        // A = Move backward, B = Strafe right
        // X = Strafe left, Y = Move forward

        if(gamepad1.y){
            // Move forward
            gp1LeftY = - Constants.ROBOT_MOVE_STRAIGHT_Y;
            gp1RightX = 0;
            gp1RightY = changeSpeed(gamepad1.left_stick_y, gp1LeftY);
        } else if(gamepad1.a) {
            // Move backward
            gp1LeftY = Constants.ROBOT_MOVE_STRAIGHT_Y;
            gp1RightX = 0;
            gp1RightY = changeSpeed(gamepad1.left_stick_y, gp1LeftY);
        } else if(gamepad1.b) {
            // Strafe Right
            double strafe = Constants.ROBOT_STRAFE_Y;
            strafe = changeSpeed(gamepad1.left_stick_x,strafe);
            strafe = Math.abs(strafe);
            gp1LeftY = - strafe;
            gp1RightX = 0;
            gp1RightY = strafe;
        } else if(gamepad1.x){
            // Strafe Left
            double strafe = Constants.ROBOT_STRAFE_Y;
            strafe = changeSpeed(gamepad1.left_stick_x,strafe);
            strafe = Math.abs(strafe);
            gp1LeftY = strafe;
            gp1RightX = 0;
            gp1RightY = - strafe;
        } else if(gamepad1.right_bumper){
            // Twist right
            double twist = Constants.ROBOT_TURN_X;
            gp1RightX = changeSpeed(gamepad1.left_stick_x,twist);
            gp1LeftY = 0;
            gp1RightY =0;
        } else if(gamepad1.left_bumper){
            // Twist left
            double twist = - Constants.ROBOT_TURN_X;
            gp1RightX = changeSpeed(gamepad1.left_stick_x,twist);
            gp1LeftY = 0;
            gp1RightY =0;
        }
    }

}