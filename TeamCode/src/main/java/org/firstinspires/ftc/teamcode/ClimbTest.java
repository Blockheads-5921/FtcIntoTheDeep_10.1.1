/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Meca
 * num drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disable
 * d line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name = "ClimbTGest", group = "Linear OpMode")
@Disabled
public class ClimbTest extends LinearOpMode {


    static final Logger log = LoggerFactory.getLogger(ClimbTest.class);

    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor boom = null;
    private DcMotor lifter = null;
    private DcMotor lowclimb = null;
    private Servo particleInOut = null;
    private Servo rightlock = null;
    private Servo leftlock = null;
    private DcMotor highclimb = null;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.


        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");

        particleInOut = hardwareMap.get(Servo.class, "sample_input");
        rightlock = hardwareMap.get(Servo.class, "rightlock");
        leftlock = hardwareMap.get(Servo.class, "leftlock");
        lifter = hardwareMap.get(DcMotor.class, "lifter");
        boom = hardwareMap.get(DcMotor.class, "boom");
        lowclimb = hardwareMap.get(DcMotor.class, "low_winch");
        highclimb = hardwareMap.get(DcMotor.class, "high_winch");

        //Lock flags
        boolean dpadUpLock = false;
        boolean dpadLeftLock = false;
        boolean dpadDownLock = false;
        boolean dpadHighLock = false;
        boolean lowClimbLock = false;

        boolean highClimbLock = false;
        boolean startClimbLock = false;
        boolean positionForClimbLock = false;
        boolean rbLock = false;
        boolean lblock = false;
        boolean reportEncoderValuesOnly = false;//nables diagnostic mode
        boolean halfPowerLock = false;
        boolean topBarLock = false;

        double halfPower = 1.0;
        int oldBoomPosition = 0;
        long startTime = 0;


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of awheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        lifter.setDirection(DcMotor.Direction.REVERSE);
        boom.setDirection(DcMotor.Direction.REVERSE);
        lowclimb.setDirection(DcMotor.Direction.FORWARD);
        highclimb.setDirection(DcMotorSimple.Direction.FORWARD);

        //Diagnostic mode
        while (reportEncoderValuesOnly) {
            telemetry.addData("Lifter Encoder ", lifter.getCurrentPosition());
            telemetry.addData("Boom Encoder", boom.getCurrentPosition());
            telemetry.addData("Low Climb Encoder ", lowclimb.getCurrentPosition());
            telemetry.addData("High Climb Encoder ", highclimb.getCurrentPosition());
            telemetry.update();
        }

        //lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setTargetPosition(0);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(0.0);

        //boom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        boom.setTargetPosition(0);
        boom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boom.setPower(0.0);

        //lowclimb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowclimb.setTargetPosition(0);
        lowclimb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lowclimb.setPower(0.0);

        //highclimb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        highclimb.setTargetPosition(0);
        highclimb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        highclimb.setPower(0.0);

        rightlock.setPosition(0.5);//0 is release, 0.5 is hold
        leftlock.setPosition(0.5);//1.0 is release, 0.5 is hold

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        int diff = 0;

        waitForStart();

        startTime = System.currentTimeMillis();
        runtime.reset();
       
            if (gamepad2.dpad_up) {
                log.info("dpadup pressed - ligter " + lifter.isBusy() + ", boom " + boom.isBusy() + ", boom pos " + boom.getCurrentPosition() + ", dpadUpLock " + dpadUpLock);
            }
            if (gamepad2.dpad_up && !dpadUpLock) {//Ok, now go to safe mode
                dpadUpLock = true;
                log.info("Safe Mode: boom position = " + boom.getCurrentPosition());
                lifter.setTargetPosition(-560);
                lifter.setPower(0.5);
                boom.setTargetPosition(100);
                boom.setPower(.99);
            } else if (!gamepad2.dpad_up && dpadUpLock && !lifter.isBusy() && !boom.isBusy()) {
                dpadUpLock = false;
            }
        }
    }



