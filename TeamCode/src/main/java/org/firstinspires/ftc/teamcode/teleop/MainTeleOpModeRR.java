package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.RobotDiagonal;
import org.firstinspires.ftc.teamcode.hardware.RobotElevator;

import java.text.DecimalFormat;

@TeleOp(name="RR - Driver Controlled")
public class MainTeleOpModeRR extends OpMode {

    // Declare Hardware object
    RobotElevator robot;
    SampleMecanumDrive sampleMecanumDrive;
    private static final PIDFController pidElevator = new PIDFController(0.001, 0d, 0d, 0d);

    public static int setElevatorHeight = 0;
    public static final int maxElevatorHeight = 3300;

    // Constructor
    public MainTeleOpModeRR() {
        // Instantiate Objects
        robot = new RobotElevator();
    }

    @Override
    public void init() {

        // Initialize Hardware
        robot.init(hardwareMap, telemetry);
        sampleMecanumDrive = new SampleMecanumDrive(hardwareMap);
        robot.setElevatorPower(0);
    }

    @Override
    public void loop() {

        //SampleMecanumDrive sampleMecanumDrive = new SampleMecanumDrive(hardwareMap);

        /* Drivetrain */
        // read inputs from gamepad

        double gp1LeftY = -gamepad1.left_stick_y;
        double gp1LeftX = -gamepad1.left_stick_x;
        double gp1RightX = -gamepad1.right_stick_x;
        boolean buttonPress = false;

        //ROBOT movements defined as
        // A = Move backward, B = Strafe right
        // X = Strafe left, Y = Move forward

        if(gamepad1.y){
            // Move forward
            gp1LeftY =  Constants.ROBOT_MOVE_STRAIGHT_Y;
            gp1RightX = 0;
            gp1LeftX = 0;
            buttonPress = true;
        } else if(gamepad1.a) {
            // Move backward
            gp1LeftY = - Constants.ROBOT_MOVE_STRAIGHT_Y;
            gp1RightX = 0;
            gp1LeftX = 0;
            buttonPress = true;
        } else if(gamepad1.b) {
            // Strafe Right
            double strafe = Constants.ROBOT_STRAFE_Y;
            strafe = changeSpeed(gamepad1.left_stick_x,strafe);
            strafe = Math.abs(strafe);
            gp1LeftY = 0;
            gp1RightX = 0;
            gp1LeftX = - strafe;
            buttonPress = true;
        } else if(gamepad1.x){
            // Strafe Left
            double strafe = Constants.ROBOT_STRAFE_Y;
            strafe = changeSpeed(gamepad1.left_stick_x,strafe);
            strafe = Math.abs(strafe);
            gp1LeftY = 0;
            gp1RightX = 0;
            gp1LeftX = strafe;
            buttonPress = true;
        } else if(gamepad1.right_bumper){
            // Twist right
            double twist = - Constants.ROBOT_TURN_ANGLE;
            gp1RightX = 0;
            gp1LeftY = 0;
            gp1LeftX =0;
            sampleMecanumDrive.turn(Math.toRadians(twist));
        } else if(gamepad1.left_bumper){
            // Twist left
            double twist = Constants.ROBOT_TURN_ANGLE;;
            gp1RightX =0;
            gp1LeftY = 0;
            gp1LeftX =0;
            sampleMecanumDrive.turn(Math.toRadians(twist));
        }

        telemetry.addLine("Gamepad 1 : Left Y : " +gp1LeftY+",  Left X : "
                +gp1LeftX+", Right X : "+gp1RightX);

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

        telemetry.addData("Claw & Elevator Movement ","%.2f : %.2f ",clawX,elevatorY);


        // Add telemetry for the raw inputs
//        telemetry.addData("Drive: ", "(%.2f)", forward);
//        telemetry.addData("Strafe: ", "(%.2f)", right);
//        telemetry.addData("Twist: ","(%.2f)",clockwise);

        // Set a scale factor to reduce the sensitivity of the forward and twist motions
        double fwdScaleFactor = 0.8;
        double rightScaleFactor = 1.0;
        double clockwiseScaleFactor = 0.8;
        if(buttonPress){
            fwdScaleFactor = 1.0;
            clockwiseScaleFactor = 1.0;
            rightScaleFactor = 1.0;
        }

        // Calling drive to make the robot move -- Start
        sampleMecanumDrive.setWeightedDrivePower(
                new Pose2d(gp1LeftY, gp1LeftX, gp1RightX) );
        sampleMecanumDrive.update();
        Pose2d poseEstimate = sampleMecanumDrive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());

        // Calling drive to make robot move -- End

        double elevatorSlidePower = 0.0;
        if(gamepad2.y){
            //Highest point
            setElevatorHeight = Constants.HIGH_JUNCTION;
            setElevatorHeight = Math.min(setElevatorHeight,maxElevatorHeight);
            telemetry.addData("RR: MainMode, High Junction",setElevatorHeight);
        }
        if(gamepad2.x) {
            setElevatorHeight = Constants.MEDIUM_JUNCTION;
            setElevatorHeight = Math.min(setElevatorHeight,maxElevatorHeight);
        }
        if(gamepad2.a) {
            setElevatorHeight = Constants.LOW_JUNCTION;
            setElevatorHeight = Math.min(setElevatorHeight,maxElevatorHeight);
            telemetry.addData("RR: MainMode, Low Junction ", setElevatorHeight);
        }
        if(gamepad2.b){
            setElevatorHeight = Constants.GROUND_LEVEL;
            setElevatorHeight = Math.min(setElevatorHeight,maxElevatorHeight);
            telemetry.addData("RR: MainMode, Base positions ", setElevatorHeight);
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
        telemetry.addData("Elevator Power and Height, CurrentPosition ","%.2f , %d , %.2f ",
                elevatorSlidePower, setElevatorHeight,robot.getElevatorPosition());
        telemetry.update();
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



}