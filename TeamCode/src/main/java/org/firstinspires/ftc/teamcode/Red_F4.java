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
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="Red_f4", group="Autonomous")
public class Red_F4 extends LinearOpMode {

public void setLifterBoom(DcMotor boom, DcMotor lifter, int boomVal, int lifterVal){
    lifter.setTargetPosition(lifterVal);
    lifter.setPower(0.4);
    boom.setTargetPosition(boomVal);
    boom.setPower(.45);
}
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(8.5, -67.5,Math.toRadians(90)));
        Servo clamp = hardwareMap.get(Servo.class, "sample_input");
        DcMotor boom = hardwareMap.get(DcMotor.class,"boom");
        DcMotor lifter = hardwareMap.get(DcMotor.class,"lifter");

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

        waitForStart();//wait to start

        clamp.setPosition(.55);
        setLifterBoom(boom,lifter,100,600);

        //Move to high bar that will clip the specimen onto it and move back
        Action highBar = drive.actionBuilder(new Pose2d(8.5, -67.5, Math.toRadians(90)))
                .waitSeconds(1)
                .lineToY(-38)
                .build();
        Action highBarpt2 = drive.actionBuilder(new Pose2d(8.5, -41, Math.toRadians(90)))
                .waitSeconds(1)
                .build();
        //Move the three strikes
        Action pushStrikes = drive.actionBuilder(new Pose2d(8.5, -50,Math.toRadians(90)))
                .waitSeconds(1)
                .lineToY(-55)
                //.waitSeconds(1)
                .splineToConstantHeading(new Vector2d(36.5, -36), Math.toRadians(45))
                .splineToConstantHeading(new Vector2d(37,-10),Math.toRadians(0))
                //.waitSeconds(0.1)
                //.lineToY(-56)
                //.splineToConstantHeading(new Vector2d(47, -10), Math.toRadians(-90))
                //.waitSeconds(0.1)
                //.splineToConstantHeading(new Vector2d(56,-57),Math.toRadians(-90))
                //.waitSeconds(0.1)
                //.lineToY(-11.5)
                //.splineToConstantHeading(new Vector2d(56.5,-57),Math.toRadians(-90))
                .build();

        //Run the trajectories plus clamp, lifter and boom action
        setLifterBoom(boom, lifter, 281,1100);//Setup for high bar clip
        Actions.runBlocking(highBar);
        clamp.setPosition(0.3);//Open the clamp
        Actions.runBlocking(highBarpt2);
        setLifterBoom(boom, lifter, 100, 1000);//Safe position
        Actions.runBlocking(highBarpt2);
        Actions.runBlocking(pushStrikes);
    }
}
