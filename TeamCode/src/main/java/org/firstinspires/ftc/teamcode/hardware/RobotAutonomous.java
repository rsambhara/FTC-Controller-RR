package org.firstinspires.ftc.teamcode.hardware;


import android.os.Build;
import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auton.Constants;
import org.firstinspires.ftc.teamcode.auton.PIDFController;


public class RobotAutonomous {

    // Declare Constants
    //public static final double CORE_HEX_EDGES_PER_RADIAN = 288d / (2 * Math.PI);


    // Declare Actuators
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor elevator = null;
    private DcMotor claw = null;
    public Telemetry telemetry = null;


    public static int setElevatorHeight = 0;
    public static final int maxElevatorHeight = 2850;
    private static final PIDFController pidOld = new PIDFController(0.001, 0d, 0d, 0d);




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
        // back right motor is mapped to back left reference is intentional
        // we want to give power to diagonal wheels in same direction.
        // back right changed to back left
        backLeft = hardwareMap.get(DcMotor.class, "backright");
        backRight = hardwareMap.get(DcMotor.class, "backleft");

        // adding elevator and claw motors
        elevator = hardwareMap.get(DcMotor.class,"elevator");
        claw = hardwareMap.get(DcMotor.class,"claw");


//        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //end original
        elevator.setDirection(DcMotorSimple.Direction.REVERSE);
        claw.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
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
//        Log.d("RD: Vector","FrontLeft: "+speed.frontLeft +", FrontRight: "+speed.frontRight
//                +", BackLeft: "+speed.backRight +", BackRight: "+speed.backLeft);
        // apply calculated speeds to the drive motors
        frontLeft.setPower(speed.frontLeft);
        frontRight.setPower(speed.frontRight);
        backLeft.setPower(speed.backLeft);
        backRight.setPower(speed.backRight);

    }

    @Deprecated
    public void elevator(double elevate, double holdPower){
        // Here claw negative is open and positive is close
        //With the sensivity of the claw material pass the max power as 0.2
        // Elevator
        // Negative power is to go up, positive power is to come down
        // For elevator to stay stable up it should have atleast .10 power
        // While coming down it is coming down slowly if power is 0.05
        holdPower = checkElevatorDirection(elevate,holdPower);
        //Log.d("RD:elevator","elevator : "+elevate +", Hold Claw power: "+holdPower);

        elevator.setPower(elevate);
        claw.setPower(holdPower);
    }

    private double checkElevatorDirection(double elevate, double holdPower) {
        //if coming down make sure claw closes
        if(elevate < 0){
            holdPower = - 0.1;
        }
        return holdPower;
    }

    private double encodeElevator(double elevate, double holdPower) {
        //if coming down make sure claw closes
        if(elevate > 0.05){
            holdPower = 0.1;
        }
        return holdPower;
    }


    public void strafe(double speed, double distance, double timeoutS, boolean opMode){

        int frontLeftTarget;
        ElapsedTime runtime = new ElapsedTime();
        int moveCounts = (int)(distance * Constants.COUNTS_PER_INCH);
        frontLeftTarget = frontLeft.getCurrentPosition() + moveCounts;

        telemetry.addLine("move counts : "+moveCounts +", distance"+distance);
        Log.d("RD:Strafe","move counts : "+moveCounts +", distance"+distance);
        // Ensure that the opmode is still active
        if (opMode) {

            // Determine new target position, and pass to motor controller
            frontLeft.setTargetPosition(frontLeftTarget);
            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            if(distance > 0){
                // If distance is positive consider this as going right
                telemetry.addData("Strafing right , front left Right and Back left right powers ",
                        " %3f :%3f: %3f :%3f",speed,-speed,-speed,speed);
                Log.d("RD:Strafe","Strafing right : front left Right and Back left right powers : +--+ "+speed);
                frontLeft.setPower(speed);
                frontRight.setPower(-speed);
                backLeft.setPower(speed);
                backRight.setPower(-speed);
            } else {
                // If distance is negative consider this as going left
                telemetry.addData("Strafing left , front left Right and Back left right powers ",
                        " %3f :%3f: %3f :%3f",-speed,speed,speed,-speed);
                Log.d("RD:Strafe","Strafing right : front left Right and Back left right powers : -++- "+speed);
                frontLeft.setPower(-speed);
                frontRight.setPower(speed);
                backLeft.setPower(-speed);
                backRight.setPower(speed);
            }

            telemetry.update();
            Log.d("RD:Strafe","Currently position of front Left is : "
                    + frontLeft.getCurrentPosition()+", target Position is : "+frontLeftTarget);
//            try {
//                Thread.sleep(5000);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ( //(runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() ) ) {

                // Display it for the driver.
                telemetry.addData("Running to all wheels to target", frontLeftTarget);
                telemetry.addData("Currently position of front Left is ", frontLeft.getCurrentPosition());
                telemetry.update();
            }
            Log.d("RD:Strafe","Final Position is "+ frontLeft.getCurrentPosition());
            // reseting the positions
            setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Stop all motion;
            setWheelPower(0);

            // Turn off RUN_TO_POSITION
            setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void drive(double speed, double distance){

    }

    public void moveElevatorToPos(double target){
        // run to pos
        int initLiftEncPos = elevator.getCurrentPosition();
        int distToPos = (int) (Constants.COUNTS_PER_INCH_ELEVATOR * target);
        int targetPos = initLiftEncPos + distToPos;
        telemetry.addData("Moving Elevator to target position", targetPos);
        Log.d("RD: moveElevatorToPos","Moving Elevator to position by "+target);
        Log.d("RD: moveElevatorToPos","Current position by "+initLiftEncPos+", Target Position"+targetPos);
        elevator.setTargetPosition(targetPos);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double speed = Constants.AUTO_ELEVATOR_SPEED;
        if(target < 0){
            //speed = -(Constants.AUTO_ELEVATOR_SPEED);
            claw.setPower(- 0.1);
        }
        while (elevator.isBusy()) {

            elevator.setPower(Constants.AUTO_ELEVATOR_SPEED);
        }
        elevator.setPower(0.05);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void changeElevatorHeight() {
        double newHt = Math.min(maxElevatorHeight, setElevatorHeight);
        double currentPos = Double.valueOf(elevator.getCurrentPosition());
        double elePower = pidOld.calculate(currentPos,newHt);
        defaultCloseClaw(newHt);
        telemetry.addData("RD: changeElevatorHeight ", elePower);
        telemetry.addData("RD: changeElevatorHeight : CurrentPosition ",
                currentPos);
        Log.d("RD: changeElevatorHeight ", "Elevator power sent by PID is : "+elePower);
        telemetry.update();
        setElevatorPIDBehavior();
        elevator.setPower(elePower);
    }

    public void changeElevatorHeight(double newHt){
        newHt = Math.min(maxElevatorHeight, newHt);
        double elePower = pidOld.calculate(
                Double.valueOf(elevator.getCurrentPosition()),newHt);
        defaultCloseClaw(newHt);
        telemetry.addData("RD: changeElevatorHeight ", elePower);
        telemetry.addData("RD: changeElevatorHeight : CurrentPosition ",
                Double.valueOf(elevator.getCurrentPosition()));
        Log.d("RD: changeElevatorHeight ", "Elevator power sent by PID is : "+elePower);
        telemetry.update();
        setElevatorPIDBehavior();
        elevator.setPower(elePower);

    }

    private void setElevatorPIDBehavior() {
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void changeElevatorHeightByUnits(double d) {
        double curPos = elevator.getCurrentPosition();
        double elePower = pidOld.calculate(curPos,Math.min(maxElevatorHeight, curPos+d));
        defaultCloseClaw(curPos);
        telemetry.addData("RD: changeElevatorHeightByUnits ", elePower);
        Log.d("RD: changeElevatorHeightByUnits ", "Elevator power sent by PID is : "+elePower);
        setElevatorPIDBehavior();
        elevator.setPower(elePower);
    }

    private void defaultCloseClaw(double currentPosition){
        // Close claw if it is going below the low junction
        if(currentPosition < Constants.LOW_JUNCTION){
            double clawP = - 0.15d;
            claw.setPower(clawP);
            telemetry.addData("RD: defaultCloseClaw ", clawP);
            Log.d("RD: defaultCloseClaw ", "Elevator power sent by PID is : "+clawP);
            telemetry.update();
        }
    }

    @Deprecated
    public void moveElevatorToPos(double target, int initLiftEncPos){
        // run to pos
        int distToPos = (int) (Constants.COUNTS_PER_INCH_ELEVATOR * target);
        int targetPos = initLiftEncPos + distToPos;
        elevator.setTargetPosition(targetPos);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (elevator.isBusy()) {
            elevator.setPower(Constants.AUTO_ELEVATOR_SPEED);
        }
        elevator.setPower(0d);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setZeroPowerMode(DcMotor.ZeroPowerBehavior behavior){
        this.frontLeft.setZeroPowerBehavior(behavior);
        this.frontRight.setZeroPowerBehavior(behavior);
        this.backLeft.setZeroPowerBehavior(behavior);
        this.backRight.setZeroPowerBehavior(behavior);
        this.elevator.setZeroPowerBehavior(behavior);
    }
    public void setEncoderMode(DcMotor.RunMode runMode){
        this.frontLeft.setMode(runMode);
        this.frontRight.setMode(runMode);
        this.backLeft.setMode(runMode);
        this.backRight.setMode(runMode);
    }


    public void Rotate ( double speed, double angle, boolean opMode) {
        int     moveCounts;
        int     newFrontRightTarget;
        int     newFrontLeftTarget;
        int     newBackRightTarget;
        int     newBackLeftTarget;
        double  leftSpeed;
        double  rightSpeed;
        double  startHeading;
        double  targetHeading;
        double  currentHeading;
        boolean clockwise;
        // set a range for error in the steering angle
        double  errorRange = 5.0;

        // Ensure that the opmode is still active

        if (opMode) {
            // Determine if turning clockwise or counterclockwise
            if (angle > 0) {
                clockwise = true;
            } else {
                clockwise = false;
            }

            // set leftSpeed and rightSpeed based on whether angle is positive or negative
            if (clockwise) {
                // Positive angle means rotate clockwise, leftSpeed will be positive, rightSpeed will be negative
                leftSpeed = Math.abs(speed);
                rightSpeed = -Math.abs(speed);
            } else {
                leftSpeed = -Math.abs(speed);
                rightSpeed = Math.abs(speed);
            }

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(angle * Constants.COUNTS_PER_DEGREE);
            if (clockwise) {
                // left will be additive, right will be subtractive
                // corrected directions based on observing robot
                // newFrontRightTarget = frontRight.getCurrentPosition() - moveCounts;
                newFrontLeftTarget = frontLeft.getCurrentPosition() + moveCounts;
                // newBackRightTarget = backRight.getCurrentPosition() + moveCounts;
                // newBackLeftTarget = backLeft.getCurrentPosition() + moveCounts;
            } else {
                // right will be additive, left will be subtractive
                // corrected directions based on observing robot
                // newFrontRightTarget = frontRight.getCurrentPosition() + moveCounts;
                newFrontLeftTarget = frontLeft.getCurrentPosition() - moveCounts;
                // newBackRightTarget = backRight.getCurrentPosition() - moveCounts;
                // newBackLeftTarget = backLeft.getCurrentPosition() - moveCounts;
            }

            // Set Target and Turn On RUN_TO_POSITION
            // frontRight.setTargetPosition(newFrontLeftTarget);
            frontLeft.setTargetPosition(newFrontLeftTarget);
            // backLeft.setTargetPosition(newBackLeftTarget);
            // backRight.setTargetPosition(newBackRightTarget);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            // speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            // this change is becoz we did diagonal pairing.
            frontRight.setPower(rightSpeed);
            frontLeft.setPower(leftSpeed);
            backLeft.setPower(rightSpeed);
            backRight.setPower(leftSpeed);
            //isMoving = true;

            // keep looping while we are still active, and BOTH motors are running.
            while (frontLeft.isBusy()) {
                telemetry.addData("Front left turning with speed : ", leftSpeed);
                telemetry.addData("Rotation Angle distance is ", moveCounts);
                telemetry.update();
            }

            // Stop all motion;
            frontRight.setPower(0);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }

    }


    public void setElevatorEncMode(DcMotor.RunMode runMode){
        this.elevator.setMode(runMode);
    }
    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS, boolean opMode) {
        int backLeftTarget;
        int frontLeftTarget;
        int backRightTarget;
        int frontRightTarget;
        ElapsedTime runtime = new ElapsedTime();

        // Ensure that the opmode is still active
        if (opMode) {

            // Determine new target position, and pass to motor controller
            int moveCount = (int)(leftInches * Constants.COUNTS_PER_INCH);
            int moveRCount = (int)(rightInches * Constants.COUNTS_PER_INCH);
            backLeftTarget = backLeft.getCurrentPosition() + moveCount;
            frontLeftTarget = frontLeft.getCurrentPosition() + moveCount;
            backRightTarget = backRight.getCurrentPosition() + moveRCount;
            frontRightTarget = frontRight.getCurrentPosition() + moveRCount;
            backLeft.setTargetPosition(backLeftTarget);
            frontLeft.setTargetPosition(frontLeftTarget);
            backRight.setTargetPosition(backRightTarget);
            frontRight.setTargetPosition(frontRightTarget);
            Log.d("RD: encoderDrive","Left Move count & COUNTS_PER_INCH "+moveCount+", "+Constants.COUNTS_PER_INCH);
            Log.d("RD: encoderDrive","Current positions of all wheels: Left Front, back : "
                    +frontLeft.getCurrentPosition()+", "+backLeft.getCurrentPosition()
                    +", Right Front, back : "+frontRight.getCurrentPosition()+", "+backRight.getCurrentPosition());
            Log.d("RD: encoderDrive","Target positions of all wheels: Left Front, back : "
                    +frontLeftTarget+", "+backLeftTarget+", Right Front, back : "+frontRightTarget+", "+backRightTarget);

            // Turn On RUN_TO_POSITION
            this.setEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            setWheelPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ( (runtime.seconds() < timeoutS) &&
                    (backLeft.isBusy() && backRight.isBusy() &&
                            frontLeft.isBusy() && frontRight.isBusy()) ) {

                // Display it for the driver.
                telemetry.addData("Originally at backLeft, backRight, frontLeft, frontRight ",  " at %3d :%3d: %3d :%3d",
                        backLeft.getCurrentPosition(),backRight.getCurrentPosition(),
                        frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
                telemetry.addData("Running to backLeftTarget, backRightTarget, frontLeftTarget, frontRightTarget",
                        " %3d :%3d: %3d :%3d", backLeftTarget,  backRightTarget, frontLeftTarget, frontRightTarget);
                telemetry.addData("Currently at backLeft, backRight, frontLeft, frontRight ",  " at %3d :%3d: %3d :%3d",
                        backLeft.getCurrentPosition(),backRight.getCurrentPosition(),
                        frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            setWheelPower(0);

            // Turn off RUN_TO_POSITION
            setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    private void setWheelPower(double speed) {
        backLeft.setPower(Math.abs(speed));
        frontLeft.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));
    }

}