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

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auton.PIDFController;
//@Disabled
@TeleOp(name="1 - Motor PID Test", group="Linear Opmode")
public class MotorPIDTestOpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor sliderMotor = null;

    private static final PIDFController pid = new PIDFController(0.001, 0d, 0d, 0d);

    public static final int LIFT_RANGE = 1650;
    public static final int COUNTS_PER_REV = 288;
    public static final double LIFT_POWER = 0.50;
    public int NUM_OF_REV = 0;
    public int targetPos = 0;

    //SETTING MOTOR MODE

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        sliderMotor = hardwareMap.get(DcMotor.class, "elevator");
        sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //SETTING MOTOR MODE
        sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderMotor.setPower(0d);
        sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (!isStopRequested()) {

            // set Motor Power
            double motorPower = 0.0;

            if (gamepad2.x) {
                //Level-1
                targetPos = 2150;
                //NUM_OF_REV = 6;
                //targetPos = NUM_OF_REV * COUNTS_PER_REV;
            }

            if (gamepad2.y) {
                //Level-2
                targetPos = 2800;
                //NUM_OF_REV = 4;
                //targetPos = NUM_OF_REV * COUNTS_PER_REV;
            }

            if (gamepad2.b) {
                //Level-0
                targetPos = 0;
                //NUM_OF_REV = 2;
                //targetPos = NUM_OF_REV * COUNTS_PER_REV;
            }
            if (gamepad2.a) {
                //Level-0
                targetPos = 1450;
                //NUM_OF_REV = 2;
                //targetPos = NUM_OF_REV * COUNTS_PER_REV;
            }

            if(gamepad2.left_bumper){
                targetPos = targetPos - 10;
            }

            if(gamepad2.right_bumper){
                targetPos = targetPos + 10;
            }
            motorPower = pid.calculate(sliderMotor.getCurrentPosition(),targetPos);
            sliderMotor.setPower(motorPower);


            telemetry.addData("Motor Power", motorPower);
            telemetry.addData("Motor Current Position", sliderMotor.getCurrentPosition());
            telemetry.addData("Motor Set Position", targetPos);
            telemetry.update();


        }
    }
}
