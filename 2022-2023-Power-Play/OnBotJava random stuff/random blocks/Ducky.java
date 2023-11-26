package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Ducky (Blocks to Java)")
public class Ducky extends LinearOpMode {

  private Servo clawAsServo;
  private DcMotor intake;
  private DcMotor frontleft;
  private DcMotor frontrightAsDcMotor;
  private DcMotor backleft;
  private DcMotor backright;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    clawAsServo = hardwareMap.get(Servo.class, "clawAsServo");
    intake = hardwareMap.get(DcMotor.class, "intake");
    frontleft = hardwareMap.get(DcMotor.class, "front left");
    frontrightAsDcMotor = hardwareMap.get(DcMotor.class, "frontrightAsDcMotor");
    backleft = hardwareMap.get(DcMotor.class, "back left");
    backright = hardwareMap.get(DcMotor.class, "back right");

    // Put initialization blocks here.
    waitForStart();
    clawAsServo.setPosition(0);
    sleep(450);
    clawAsServo.setPosition(0.5);
    sleep(450);
  }
}
