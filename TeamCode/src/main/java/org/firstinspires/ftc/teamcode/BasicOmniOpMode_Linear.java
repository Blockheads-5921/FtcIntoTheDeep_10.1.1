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

@TeleOp(name = "BasicOpMode_Linear", group = "Linear OpMode")

public class BasicOmniOpMode_Linear extends LinearOpMode {


    static final Logger log = LoggerFactory.getLogger(BasicOmniOpMode_Linear.class);

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
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

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

        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setTargetPosition(0);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(0.0);

        boom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        boom.setTargetPosition(0);
        boom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boom.setPower(0.0);

        lowclimb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowclimb.setTargetPosition(0);
        lowclimb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lowclimb.setPower(0.0);

        highclimb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        highclimb.setTargetPosition(0);
        highclimb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        highclimb.setPower(0.0);

        rightlock.setPosition(0.5);//0 is release, 0.5 is hold
        leftlock.setPosition(0.5);//1.0 is release, 0.5 is hold

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            double max = 0.0;

            //set power to full or half power for drive motors
            if (!halfPowerLock && gamepad1.right_bumper) {
                halfPowerLock = true;
                if (halfPower == 1.0) {
                    halfPower = 0.5;
                } else if (halfPower == 0.5) {
                    halfPower = 1.0;
                }
            } else if (halfPowerLock && !gamepad1.right_bumper) {
                halfPowerLock = false;
            }

            // Input servo
            //

            if (gamepad2.right_bumper) {
                log.info("KKKK Right bumper - Grip");
                particleInOut.setPosition(0.55);  //Grip position
            } else if (gamepad2.left_bumper) {
                log.info("kkkk Left bumper - Release");
                particleInOut.setPosition(0.3);  //Open position
            }

            //diagnostic info
            telemetry.addData("Lifter Encoder ", lifter.getCurrentPosition());
            telemetry.addData("Boom Encoder", boom.getCurrentPosition());
            telemetry.addData("Low Climb Encoder ", lowclimb.getCurrentPosition());
            telemetry.addData("High Climb Encoder ", highclimb.getCurrentPosition());


            //Go to safe position
            if (gamepad2.dpad_up) {//First determine if the boom is stalled
                log.info("FTCRobot -- dpad_up pressed, Lifter " + lifter.isBusy() + ",  Boom " +
                        boom.isBusy() + " Boom Pos " + boom.getCurrentPosition()  + dpadUpLock);
                if(boom.isBusy()){
                    int diff = Math.abs(oldBoomPosition-boom.getCurrentPosition());
                    if (diff==0){
                        log.info("Boom RESET, stalled at " + boom.getCurrentPosition() +
                                " ***********************");
                        boom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        oldBoomPosition = 0;
                        dpadUpLock = false;
                    }else {
                        oldBoomPosition = boom.getCurrentPosition();
                    }
                }
            }
            if (gamepad2.dpad_up && !dpadUpLock) {//Ok, now go to safe mode
                dpadUpLock = true;
                lifter.setTargetPosition(600);
                lifter.setPower(0.4);
                boom.setTargetPosition(100);
                boom.setPower(.99);
            } else if (!gamepad2.dpad_up && dpadUpLock && !lifter.isBusy() && !boom.isBusy()) {
                dpadUpLock = false;
            }

            //Specimen top bar position
            if (gamepad2.x) {
                log.info("FTCRobot -- X pressed, Lifter " + lifter.isBusy() + ", Boom " + boom.isBusy() + ",  topBarLock " + topBarLock);
            }
            if (gamepad2.x && !topBarLock && !lifter.isBusy() && !boom.isBusy()) {
                topBarLock = true;
                boom.setTargetPosition(281);
                boom.setPower(0.45);
                lifter.setTargetPosition(1175);
                lifter.setPower(0.45);
            } else if (!gamepad2.x && topBarLock && !lifter.isBusy() && !boom.isBusy()) {
                topBarLock = false;
            }

            //Set 24" position
            if (gamepad2.dpad_down) {
                log.info("FTCRobot -- dpad_down pressed, Lifter " + lifter.isBusy() + ", Boom " + boom.isBusy() + ",  dpadDownLock " + dpadDownLock);
            }
            if (gamepad2.dpad_down && !dpadDownLock && !lifter.isBusy() && !boom.isBusy()) {
                dpadDownLock = true;
                log.info("FTCRobot -- 24 inch section");
                lifter.setTargetPosition(490);
                lifter.setPower(0.2);
                boom.setTargetPosition(2300);
                boom.setPower(.99);
            } else if (!gamepad2.dpad_down && dpadDownLock && !lifter.isBusy() && !boom.isBusy()) {
                lifter.setTargetPosition(297);
                lifter.setPower(.2);
                dpadDownLock = false;
            }

            //High basket
            if (gamepad2.dpad_right) {
                log.info("FTCRobot -- dpad_right pressed, Lifter  " + lifter.isBusy() + ", Boom " + boom.isBusy() + ",  dpadHighLock " + dpadHighLock);
            }
            if (gamepad2.dpad_right && !dpadHighLock && !lifter.isBusy() && !boom.isBusy()) {
                dpadHighLock = true;
                lifter.setTargetPosition(1520);
                lifter.setPower(0.2);
                boom.setTargetPosition(2100);
                boom.setPower(.99);
            } else if (!gamepad2.dpad_right && dpadHighLock && !lifter.isBusy() && !boom.isBusy()) {
                dpadHighLock = false;
            }

            //Position lifter for climb to low bar
            if (gamepad2.a) {
                log.info("FTCRobot -- A button pressed, Lifter " + lifter.isBusy() + ", Boom " + boom.isBusy() + ",  positionForClimbLock " + positionForClimbLock);
            }
            if (gamepad2.a && !positionForClimbLock && !lifter.isBusy() && !boom.isBusy()) {
                positionForClimbLock = true;
                boom.setTargetPosition(100);
                boom.setPower(.45);
                lifter.setTargetPosition(1960);
                lifter.setPower(0.45);
            } else if (!gamepad2.a && positionForClimbLock && !lifter.isBusy() && !boom.isBusy()) {
                positionForClimbLock = false;
            }

            //Synchronize high climb rope with the boom as it moves in and out
            if (!highClimbLock) {
                highclimb.setTargetPosition((int) Math.round(boom.getCurrentPosition() * 1.9));
                highclimb.setPower(100.0);


                // Climb to low bar
                if (gamepad2.b) {
                    log.info("FTCRobot -- B pressed, Lifter " + lifter.isBusy() + ", Boom " + boom.isBusy() + ",  startClimbLock " + startClimbLock);
                }
                if (gamepad2.b && !startClimbLock) {
                    startClimbLock = true;
                    lowclimb.setTargetPosition(7600);
                    lowclimb.setPower(100.0);
                }

                //Move lifter during climb to low bar so that is does not hit the floor
                if (startClimbLock && (lowclimb.getCurrentPosition() > 5600) && !lowClimbLock) {
                    lowClimbLock = true;
                    lifter.setTargetPosition(4200);
                    lifter.setPower(0.5);
                    boom.setTargetPosition(100);//2600
                    boom.setPower(0.45);
                }
/*
            //Start climb to upper bar
                log.info("FTCRobot -- Climb to upper bar: lowClimblock " + lowClimbLock +", lifter busy " + lifter.isBusy() + ", boom busy " + boom.isBusy());
                if ( startClimbLock && lowClimbLock && !lifter.isBusy() && !boom.isBusy()){
                    highClimbLock  = true;
                    highclimb.setTargetPosition(-6000);
                    highclimb.setPower(1.00);
                }
                //Get the boom and lifter out of the way while the climb to the top bar is made
                if (highClimbLock && (highclimb.getCurrentPosition() < -3000)){
                    log.info("FTCRobot -- High climb: Get lifter and boom out of the way");
                    boom.setTargetPosition(100);
                    boom.setPower(1.00);
                    lifter.setTargetPosition(1960);
                    lifter.setPower(0.2);
                }
                //Unhook from the low bar
                if (highClimbLock && highclimb.getCurrentPosition() < -4000){
                    log.info("FTCRobot --  High climb Unhook");
                    rightlock.setPosition(0.0);
                    leftlock.setPosition(1.0);
                    lowclimb.setTargetPosition(4800);
                    lowclimb.setPower(1.00);
                }
*/

                /**/
                //Macanum drive section

                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                double axial = -gamepad1.left_stick_y * halfPower;  // Note: pushing stick forward gives negative value
                double lateral = gamepad1.left_stick_x * halfPower; // gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x * halfPower; //gamepad1.right_stick_x;


                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                // Set up a variable for each drive wheel to save the power level for telemetry.
                double leftFrontPower = axial + lateral + yaw;
                double rightFrontPower = axial - lateral - yaw;
                double leftBackPower = axial - lateral + yaw;
                double rightBackPower = axial + lateral - yaw;

                // Normalize the values so no wheel power exceeds 100%
                // This ensures that the robot maintains the desired motion.
                max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));

                if (max > 1.0) {
                    leftFrontPower /= max;
                    rightFrontPower /= max;
                    leftBackPower /= max;
                    rightBackPower /= max;
                }

                // This is test code:
                //
                // Uncomment the following code to test your motor directions.
                // Each button should make the corresponding motor run FORWARD.
                //   1) First get all the motors to take to correct positions on the robot
                //      by adjusting your Robot Configuration if necessary.
                //   2) Then make sure they run in the correct direction by modifying the
                //      the setDirection() calls above.
                // Once the correct motors move in the correct direction re-comment this code.

                /*
                leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
                leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
                rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
                rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
                */

                /*
                wireless:
                1. turn robot on
                2. connect to robot wifi
                3. open REV Hardware Client
                4. something about control hubs should appear instead of medium phone API
                5. press the green play button (it might be a bent arrow shape)
                 */


                // Send calculated power to wheels
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);

                // Show the elapsed game time and wheel power.
                //    telemetry.addData("Status", "Run Time: " + runtime.toString());
                //    telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
                //    telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
                telemetry.update();
            }
        }
    }
}
