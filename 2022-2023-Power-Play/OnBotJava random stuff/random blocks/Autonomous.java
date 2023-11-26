package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Autonomous (Blocks to Java)")
@Disabled
public class Autonomous extends LinearOpMode {

  private Servo fingerAsServo;
  private DcMotor frontLeftAsDcMotor;
  private DcMotor backLeftAsDcMotor;
  private DcMotor frontRightAsDcMotor;
  private DcMotor backRightAsDcMotor;
  private DcMotor flywheelAsDcMotor;
  private DcMotor armAsDcMotor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    fingerAsServo = hardwareMap.get(Servo.class, "fingerAsServo");
    frontLeftAsDcMotor = hardwareMap.get(DcMotor.class, "frontLeftAsDcMotor");
    backLeftAsDcMotor = hardwareMap.get(DcMotor.class, "backLeftAsDcMotor");
    frontRightAsDcMotor = hardwareMap.get(DcMotor.class, "frontRightAsDcMotor");
    backRightAsDcMotor = hardwareMap.get(DcMotor.class, "backRightAsDcMotor");
    flywheelAsDcMotor = hardwareMap.get(DcMotor.class, "flywheelAsDcMotor");
    armAsDcMotor = hardwareMap.get(DcMotor.class, "armAsDcMotor");

    // Put initialization blocks here.
    fingerAsServo.setPosition(1);
    frontLeftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    backLeftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    frontRightAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    backRightAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    flywheelAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    waitForStart();
    frontLeftAsDcMotor.setPower(0.8);
    backLeftAsDcMotor.setPower(0.8);
    frontRightAsDcMotor.setPower(0.8);
    backRightAsDcMotor.setPower(0.8);
    sleep(600);
    frontLeftAsDcMotor.setPower(0);
    frontRightAsDcMotor.setPower(0);
    backLeftAsDcMotor.setPower(0);
    backRightAsDcMotor.setPower(0);
    flywheelAsDcMotor.setPower(0.9);
    sleep(2250);
    fingerAsServo.setPosition(0.81);
    sleep(300);
    fingerAsServo.setPosition(1);
    sleep(500);
    fingerAsServo.setPosition(0.81);
    sleep(500);
    fingerAsServo.setPosition(1);
    sleep(500);
    fingerAsServo.setPosition(0.81);
    sleep(500);
    fingerAsServo.setPosition(1);
    sleep(500);
    flywheelAsDcMotor.setPower(0);
    frontLeftAsDcMotor.setPower(0.8);
    frontRightAsDcMotor.setPower(0.8);
    backLeftAsDcMotor.setPower(0.8);
    backRightAsDcMotor.setPower(0.8);
    sleep(150);
    frontLeftAsDcMotor.setPower(0);
    frontRightAsDcMotor.setPower(0);
    backLeftAsDcMotor.setPower(0);
    backRightAsDcMotor.setPower(0);
    armAsDcMotor.setPower(0.9);
    sleep(1000);
    armAsDcMotor.setPower(0);
  }
}
