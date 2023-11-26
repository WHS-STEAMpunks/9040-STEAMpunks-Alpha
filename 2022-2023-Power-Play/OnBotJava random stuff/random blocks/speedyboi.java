package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "speedyboi (Blocks to Java)")
@Disabled
public class speedyboi extends LinearOpMode {

  private DcMotor frontLeftAsDcMotor;
  private DcMotor frontRightAsDcMotor;
  private DcMotor backLeftAsDcMotor;
  private DcMotor backRightAsDcMotor;
  private DcMotor intake;
  private DcMotor flywheelAsDcMotor;
  private DcMotor armAsDcMotor;
  private Servo fingerAsServo;
  private Servo clawAsServo;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    frontLeftAsDcMotor = hardwareMap.get(DcMotor.class, "frontLeftAsDcMotor");
    frontRightAsDcMotor = hardwareMap.get(DcMotor.class, "frontRightAsDcMotor");
    backLeftAsDcMotor = hardwareMap.get(DcMotor.class, "backLeftAsDcMotor");
    backRightAsDcMotor = hardwareMap.get(DcMotor.class, "backRightAsDcMotor");
    intake = hardwareMap.get(DcMotor.class, "intake");
    flywheelAsDcMotor = hardwareMap.get(DcMotor.class, "flywheelAsDcMotor");
    armAsDcMotor = hardwareMap.get(DcMotor.class, "armAsDcMotor");
    fingerAsServo = hardwareMap.get(Servo.class, "fingerAsServo");
    clawAsServo = hardwareMap.get(Servo.class, "clawAsServo");

    // Put initialization blocks here.
    frontLeftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    frontRightAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    backLeftAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    backRightAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    intake.setDirection(DcMotorSimple.Direction.FORWARD);
    flywheelAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    armAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    fingerAsServo.setPosition(0.9);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        if (gamepad2.left_bumper) {
          armAsDcMotor.setPower(0.9);
        } else {
          armAsDcMotor.setPower(0);
        }
        if (gamepad2.right_bumper) {
          armAsDcMotor.setPower(-0.7);
        } else {
          armAsDcMotor.setPower(0);
        }
        if (gamepad2.a) {
          intake.setPower(0.9);
        } else {
          intake.setPower(0);
        }
        if (gamepad2.y) {
          clawAsServo.setPosition(0.3);
        } else {
          clawAsServo.setPosition(0.9);
        }
        if (gamepad2.dpad_up) {
          flywheelAsDcMotor.setPower(0.9);
        } else {
          flywheelAsDcMotor.setPower(0);
        }
        if (gamepad2.x) {
          fingerAsServo.setPosition(0.81);
        } else {
          fingerAsServo.setPosition(1);
        }
        if (gamepad1.right_bumper) {
          frontLeftAsDcMotor.setPower(0.9);
          frontRightAsDcMotor.setPower(0.9);
          backLeftAsDcMotor.setPower(0.9);
          backRightAsDcMotor.setPower(0.9);
        } else {
          frontLeftAsDcMotor.setPower(0);
          frontRightAsDcMotor.setPower(0);
          backLeftAsDcMotor.setPower(0);
          backRightAsDcMotor.setPower(0);
        }
        if (gamepad1.left_bumper) {
          frontLeftAsDcMotor.setPower(-0.9);
          frontRightAsDcMotor.setPower(-0.9);
          backLeftAsDcMotor.setPower(-0.9);
          backRightAsDcMotor.setPower(-0.9);
        } else {
          frontLeftAsDcMotor.setPower(0);
          frontRightAsDcMotor.setPower(0);
          backLeftAsDcMotor.setPower(0);
          backRightAsDcMotor.setPower(0);
        }
        if (gamepad1.left_stick_y < -0.1) {
          frontLeftAsDcMotor.setPower(0.9);
          frontRightAsDcMotor.setPower(-0.9);
          backLeftAsDcMotor.setPower(-0.9);
          backRightAsDcMotor.setPower(0.9);
        } else {
          frontLeftAsDcMotor.setPower(0);
          frontRightAsDcMotor.setPower(0);
          backLeftAsDcMotor.setPower(0);
          backRightAsDcMotor.setPower(0);
        }
        if (gamepad1.left_stick_y > 0.1) {
          frontLeftAsDcMotor.setPower(-0.9);
          frontRightAsDcMotor.setPower(0.9);
          backLeftAsDcMotor.setPower(0.9);
          backRightAsDcMotor.setPower(-0.9);
        } else {
          frontLeftAsDcMotor.setPower(0);
          frontRightAsDcMotor.setPower(0);
          backLeftAsDcMotor.setPower(0);
          backRightAsDcMotor.setPower(0);
        }
        if (gamepad1.right_stick_x < -0.1) {
          frontLeftAsDcMotor.setPower(-0.9);
          frontRightAsDcMotor.setPower(-0.9);
          backLeftAsDcMotor.setPower(0.9);
          backRightAsDcMotor.setPower(0.9);
        } else {
          frontLeftAsDcMotor.setPower(0);
          frontRightAsDcMotor.setPower(0);
          backLeftAsDcMotor.setPower(0);
          backRightAsDcMotor.setPower(0);
        }
        if (gamepad1.right_stick_x > 0.1) {
          frontLeftAsDcMotor.setPower(0.9);
          frontRightAsDcMotor.setPower(0.9);
          backLeftAsDcMotor.setPower(-0.9);
          backRightAsDcMotor.setPower(-0.9);
        } else {
          frontLeftAsDcMotor.setPower(0);
          frontRightAsDcMotor.setPower(0);
          backLeftAsDcMotor.setPower(0);
          backRightAsDcMotor.setPower(0);
        }
        telemetry.update();
      }
    }
  }
}
