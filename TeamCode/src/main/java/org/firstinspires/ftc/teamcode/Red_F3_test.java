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

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Autonomous(name = "Red_f4_Test", group = "Autonomous")
public class Red_F3_test extends LinearOpMode {

    public void setLifterBoom(DcMotor boom, DcMotor lifter, int boomVal, int lifterVal) {
        lifter.setTargetPosition(lifterVal);
        lifter.setPower(1);
        boom.setTargetPosition(boomVal);
        boom.setPower(1);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(8.5, -67.5, Math.toRadians(90)));
        Servo clamp = hardwareMap.get(Servo.class, "sample_input");
        DcMotor boom = hardwareMap.get(DcMotor.class, "boom");
        DcMotor lifter = hardwareMap.get(DcMotor.class, "lifter");

        boom.setDirection(DcMotorSimple.Direction.REVERSE);
        lifter.setDirection(DcMotorSimple.Direction.REVERSE);

        boom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        boom.setTargetPosition(0);
        boom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boom.setPower(0.0);

        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setTargetPosition(0);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(0.0);

        final double GRAB = 0.55;
        final double RELEASE =0.3;



        //Move to high bar that will clip the specimen onto it and move back
        Action highBar = drive.actionBuilder(new Pose2d(8.5, -64.5, Math.toRadians(90)))
                .lineToY(-37, new TranslationalVelConstraint(11))
                .build();
        Action backToMinus50 = drive.actionBuilder(new Pose2d(8.5, -37, Math.toRadians(90)))
                .lineToY(-50)
                .build();



        //Get strike 1
        Action getStrike1 = drive.actionBuilder(new Pose2d(8.5, -50, Math.toRadians(90)))
                //.setReversed(false)
                .splineTo(new Vector2d(22.5, -45), Math.toRadians(28))
                .build();

        //Drop strike 1
        Action dropStrike1 = drive.actionBuilder( new Pose2d(22.5, -45, Math.toRadians(28)))
                .splineTo(new Vector2d(34.4, -47.3), Math.toRadians(-70))
                .build();

        //Get strike 2
        Action getStrike2 = drive.actionBuilder( new Pose2d(34.4, -47.3, Math.toRadians(-70)))
                .splineTo(new Vector2d(34.4, -45), Math.toRadians(51.1),
                        new TranslationalVelConstraint(10))
                .build();

        //Drop strike 2
        Action dropStrike2 = drive.actionBuilder( new Pose2d(31, -47.5, Math.toRadians(51.1)))
                .splineTo(new Vector2d(44.2, -47.5), Math.toRadians(-41.7))
                .build();
/*
        //Get specimen 1 from wall
        Action getSpecimen1= drive.actionBuilder( new Pose2d(44.2, -47.5, Math.toRadians(-41.7)))
                .splineTo(new Vector2d(56.2, -57), Math.toRadians(90))//get it
                .build();


        //Clip specimen 1
        Action clipSpecimen1 = drive.actionBuilder( new Pose2d(56.2, -57, Math.toRadians(-90)))
                .splineTo(new Vector2d(0, -38), Math.toRadians(-90))
                .build();

        //Back away from the wall
        Action backAway0 = drive.actionBuilder( new Pose2d(0, -38, Math.toRadians(-90)))
                .lineToY(-45)
                .build();

        //Get specimen 2
        Action getSpecimen2 = drive.actionBuilder( new Pose2d(0, -45, Math.toRadians(-90)))
                .splineTo(new Vector2d(42, -57), Math.toRadians(-90))
                .build();

        //Clip specimen 2
        Action clipSpecimen2 = drive.actionBuilder( new Pose2d(42, -57, Math.toRadians(-90)))
                .splineTo(new Vector2d(2, -38), Math.toRadians(90))
                .build();

        //Back away from x = 2
        Action backAway2 = drive.actionBuilder( new Pose2d(2, -38, Math.toRadians(-90)))
                .lineToY(-45)
                .build();

        //Park
        Action park = drive.actionBuilder( new Pose2d(2, -45, Math.toRadians(0)))
                .splineTo(new Vector2d(42, -55), Math.toRadians(0))
                .build();

*/
        waitForStart();//wait to start


        //==========================  Beginning of autonomous  =====================================

        //Clip preloaded specimen
        clamp.setPosition(GRAB);
        Thread.sleep(150);
        setLifterBoom(boom, lifter, 281, 1045);
        //while (boom.isBusy() || lifter.isBusy()){ }
        Actions.runBlocking(highBar);
        clamp.setPosition(RELEASE);
        Thread.sleep(150);
        Actions.runBlocking(backToMinus50);


        //Get strike 1
        setLifterBoom(boom, lifter, 2250, 490);//get ready clamp strike 1
        Actions.runBlocking(getStrike1);
        while (boom.isBusy()){}
        setLifterBoom(boom, lifter, 2250, 297);
        Thread.sleep(500);
        clamp.setPosition(GRAB);
        Thread.sleep(250);

        //Drop strike 1
        setLifterBoom(boom, lifter, 2300, 490);
        while (lifter.isBusy() || lifter.isBusy()){ }
        Actions.runBlocking(dropStrike1);
        clamp.setPosition(RELEASE);
        setLifterBoom(boom, lifter, 1500, 297);
        Thread.sleep(500);

        //Get strike 2
        setLifterBoom(boom, lifter, 2250, 490);
        while (lifter.isBusy() || boom.isBusy()){}
        Actions.runBlocking(getStrike2);
        //Thread.sleep(1250);
        setLifterBoom(boom, lifter, 2200, 297);
        while (lifter.isBusy() || boom.isBusy()){}
        clamp.setPosition(GRAB);
        Thread.sleep(250);


        //Drop strike 2
        //Actions.runBlocking(dropStrike2);
        //clamp.setPosition(RELEASE);
/*
        //Get specimen 1 from wall
        setLifterBoom(boom, lifter, 100, 545);
        while (boom.isBusy() || lifter.isBusy()){ }
        Actions.runBlocking(getSpecimen1);
        clamp.setPosition(GRAB);
        Thread.sleep(500);

        //clip specimen 1
        setLifterBoom(boom, lifter, 281, 1025);
        Actions.runBlocking(clipSpecimen1);
        clamp.setPosition(RELEASE);

        //Back away from the wall
        Actions.runBlocking(backAway0);
        setLifterBoom(boom, lifter,100, 545);

        //Get specimen 2
        Actions.runBlocking(getSpecimen2);
        clamp.setPosition(GRAB);
        Thread.sleep(250);
        setLifterBoom(boom, lifter, 281, 1025);

        //Go clip specimen 2
        Actions.runBlocking(clipSpecimen2);

        //Back away
        Actions.runBlocking(backAway2);
        setLifterBoom(boom, lifter,100, 545);

        //Go park
        setLifterBoom(boom, lifter, 490, 2300);
        Actions.runBlocking(park);
*/
    }
}
