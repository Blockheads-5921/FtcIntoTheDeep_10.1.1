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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "Basket", group = "Autonomous")
public class Basket extends LinearOpMode {

    public void setLifterBoom(DcMotor boom, DcMotor lifter, int boomVal, int lifterVal) {
        lifter.setTargetPosition(lifterVal);
        lifter.setPower(.45);
        boom.setTargetPosition(boomVal);
        boom.setPower(.45);
    }

    public void setLifterBoomAndWait(DcMotor boom, DcMotor lifter, int boomVal, int lifterVal) {
        lifter.setTargetPosition(lifterVal);
        lifter.setPower(1);
        boom.setTargetPosition(boomVal);
        boom.setPower(1);
        while (boom.isBusy() || lifter.isBusy()){}
    }
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-8.5, -67.5, Math.toRadians(90)));
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
        final double waitTime = 1.0;



        //Move to high bar that will clip the specimen onto it and move back
        Action highBar = drive.actionBuilder(new Pose2d(-8.5, -64.5, Math.toRadians(90)))
                .lineToY(-34, new TranslationalVelConstraint(11))
                .build();

        //position to grab strike
        Action getRightStrike = drive.actionBuilder(new Pose2d(-8.5, -35, Math.toRadians(90)))
                .lineToY(-60)
                .splineToSplineHeading(new Pose2d(-21, -41, Math.toRadians(173)), Math.toRadians(123))
                .build();

        //Go to the basket
        Action goToBasket = drive.actionBuilder(new Pose2d(-21, -41, Math.toRadians(173)))
                .splineTo(new Vector2d(-58, -58), Math.toRadians(-103))
                .build();

        //Backup and drop boom
        Action backup = drive.actionBuilder(new Pose2d(-57, -57, Math.toRadians(-103)))
                .setReversed(false)
                .splineTo(new Vector2d(-43, -43), Math.toRadians(-133))
                .build();

        //Park
        Action park = drive.actionBuilder(new Pose2d(-43, -43, Math.toRadians(-133)))
                .setReversed(false)
                .splineTo(new Vector2d(-54, -60), Math.toRadians(-130))
                .build();


        waitForStart();//wait to start


        //==========================  Beginning of autonomous  =====================================

        //Clip preloaded specimen
        clamp.setPosition(GRAB);
        //Actions.runBlocking(delay1);
        setLifterBoom(boom, lifter, 281, 1025);
        //while (boom.isBusy() || lifter.isBusy()){ }
        Actions.runBlocking(highBar);
        clamp.setPosition(RELEASE);

        //Drop back then extend boom
         Actions.runBlocking(getRightStrike);
         //Thread.sleep(1000);
         setLifterBoom(boom, lifter,2350, 297);
         while (boom.isBusy() || lifter.isBusy()){};
         clamp.setPosition(GRAB);
         Thread.sleep(1000);

         //Go to Basket
        setLifterBoom(boom, lifter, 2300, 1520);
        while (boom.isBusy() || lifter.isBusy()){}
        Thread.sleep(500);
        Actions.runBlocking(goToBasket);
        clamp.setPosition(RELEASE);
        Thread.sleep(1000);

        //Backup and drop boom
        Actions.runBlocking(backup);
        setLifterBoom(boom, lifter, 100, 565);
        while (boom.isBusy() || lifter.isBusy()){};

        //Park
        Actions.runBlocking(park);




    }
}
