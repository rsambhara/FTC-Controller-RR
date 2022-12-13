/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.autotests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="Robot: TestAutonomous", group="Robot")
@Disabled
public class AutonomousTest extends LinearOpMode {

    protected DcMotor frontLeft;
    protected DcMotor frontRight;
    protected DcMotor backLeft;
    protected DcMotor backRight;
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    //public Telemetry telemetry = null;

    public void runOpMode() throws InterruptedException {

        //telemetry = n
        // added Telemetry
        telemetry.addData("Status Initialized", runtime.toString());
//        telemetry.update();

        // start original
        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        //This was intentionally changed to have the desired movement of the
        // diagonal wheels in the same direction. So this was flipped
        // backright to backleft
        backLeft = hardwareMap.get(DcMotor.class, "backright");
        backRight = hardwareMap.get(DcMotor.class, "backleft");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        double powerWheels = 0.4;

        // Encode Code Start

        telemetry.addData("Power passed is moving : ",powerWheels);
        telemetry.addData("Time in seconds", runtime.seconds());
        telemetry.update();

//        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        moveRobot(powerWheels, powerWheels, powerWheels, powerWheels, 5000); //FORWARD

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        powerWheels = 0;
        telemetry.addData("Power passed is stopped : ",powerWheels);
//        telemetry.update();
        moveRobot(powerWheels, powerWheels, powerWheels, powerWheels, 1000); //FORWARD
        telemetry.addLine("Brake applied");
        telemetry.update();


//        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Encoder Code End
        telemetry.update();
        waitForStart();

        moveRobot(0.2, 0.2, 0.2, 0.2, 1000); //FORWARD
//        moveRobot(-0.2, -0.2, -0.2, -0.2, 1000); //BACKWARD
//        moveRobot(0, 0.2, 0.2, 0, 1000); //DIAGONAL, FRONT LEFT
//        moveRobot(0.2, 0, 0, 0.2, 1000); //DIAGONAL, FRONT RIGHT
//        moveRobot(-0.2, 0.2, 0.2, -0.2, 1000); //STRAFE, LEFT
//        moveRobot(0.2, -0.2, -0.2, 0.2, 1000); //STRAFE, RIGHT
//        moveRobot(-0.2, -0.2, -0.2, -0.2, 1000); //DIAGONAL, BACK LEFT
//        moveRobot(-0.2, 0, 0, -0.2, 1000); //DIAGONAL, BACK RIGHT
//        moveRobot(0.2, -0.2, 0.2, -0.2, 1000); //TURN, CLOCKWISE
//        moveRobot(-0.2, 0.2, -0.2, -0.2, 1000); //TURN, ANTICLOCKWISE

    }

    public void moveRobot(double powerfl, double powerfr, double powerbl, double powerbr, int time) {

        frontLeft.setPower(powerfl);
        frontRight.setPower(powerfr);
        backLeft.setPower(powerbl);
        backRight.setPower(powerbr);
        sleep(time);

    }

}