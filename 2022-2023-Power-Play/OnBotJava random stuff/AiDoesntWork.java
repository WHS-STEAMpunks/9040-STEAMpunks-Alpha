package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "aidoesntwork")

public class AiDoesntWork extends LinearOpMode{
  
  private DcMotorEx frontleft;
  private DcMotorEx backright;
  private DcMotorEx backleft;
  private DcMotorEx frontright;
  private DcMotor slideExtender;
  private Servo turner;
  private Servo claw;
  private final static double TURNER_HOME = .7;
  private final static double TURNER_MIN_RANGE =.65;
  private final static double TURNER_MAX_RANGE = .7; 
  private final static double CLAW_HOME = .3;
  private final static double CLAW_MIN_RANGE = .65;
  private final static double CLAW_MAX_RANGE = .8;
  private final double TURNER_SPEED = 0.0001;
  private  final double CLAW_SPEED = 0.1;
  private static final double COUNTS_PER_MOTOR_REV = 537.7;
  private static final double DRIVE_GEAR_REDUCTION = 19.2;
  private static final double WHEEL_DIAMETER_INCHES = 4.0;
  private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION)/WHEEL_DIAMETER_INCHES;
  
  // todo: write your code here
@Override
  public void runOpMode() {//initializes motors and servos
    frontleft = hardwareMap.get(DcMotorEx.class, "front left");
    backleft = hardwareMap.get(DcMotorEx.class, "back left");
    backright = hardwareMap.get(DcMotorEx.class, "back right");
    frontright = hardwareMap.get(DcMotorEx.class, "front right");
    slideExtender = hardwareMap.get(DcMotor.class, "slide extender");
    turner = hardwareMap.get(Servo.class, "turner");
    claw = hardwareMap.get(Servo.class, "claw");
    
    double turnerPosition = TURNER_HOME;
    double clawPosition = CLAW_HOME;
    double clawSpeed = 0;
    double turnerSpeed = 0;
    double motorVelocity = 400;//Set Motor speed
    frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); //Sets to encoder mode
    frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    //zero power behavior
    backleft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    backright.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    frontleft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    frontright.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    waitForStart();
    //begin ai mode
    while(opModeIsActive())
    {
      
    }

    }
    
      private void encodeDrive(double fl,double fr,double bl,double br,double power)
      {
        int newfl;
        int newfr;
        int newbl;
        int newbr;
        if(opModeIsActive())
        {
            //reset encoder program
          frontleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
          frontright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
          backleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
          backright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
          //determines new rotation distance
          newfl = (int)(fl*COUNTS_PER_INCH);
          newfr = (int)(fr*COUNTS_PER_INCH);
          newbl = (int)(bl*COUNTS_PER_INCH);
          newbr = (int)(br*COUNTS_PER_INCH);
            //sets how many times the robot rotates
          frontleft.setTargetPosition(newfl);
          frontright.setTargetPosition(newfr);
          backleft.setTargetPosition(newbl);
          backright.setTargetPosition(newbr);
          //rotates to position
          frontleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
          frontright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
          backleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
          backright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
          //rotate mode
          frontleft.setPower(power);
          frontright.setPower(power);
          backleft.setPower(power);
          backright.setPower(power);
          while (opModeIsActive() && (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy()
           && backright.isBusy()))
           {
               /*
               1. Line is very important in keeping the motors busy and tracking the number of ticks.
                */
           }
          
           frontleft.setPower(0);
           frontright.setPower(0);
           backleft.setPower(0);
           backright.setPower(0);
           /*
           1. Stop all movement.
            */

           frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
           frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
           backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
           backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
}
        }
        private void moveForward(double distance, double power){
            encodeDrive(-distance, distance, -distance, distance, power/2);
        }
        private void moveBackward(double distance, double power){
            moveForward(-distance, power);
        }
        private void strafeLeft(double distance, double power){
            encodeDrive(-distance, distance, distance, -distance, power);
        }
        private void strafeRight(double distance, double power){
            strafeLeft(-distance, power);
        }
        private void turnLeft(double distance,double power)
        {
            encodeDrive(distance, distance, distance, distance, power);
        }
        private void turnRight(double distance,double power)
        {
            turnLeft(-distance, power);
        }
        
        
        
        
        
        
        
        
        
        
        
}