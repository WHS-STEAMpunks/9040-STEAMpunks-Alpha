/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "KinderCode")


public class KinderCode extends LinearOpMode 
{
    private DcMotor frontleft;
    private DcMotor backright;
    private DcMotor backleft;
    private DcMotor frontright;
    private ColorSensor colorsensor;

  
    @Override
    public void runOpMode() 
    {
        frontleft = hardwareMap.get(DcMotor.class, "front left");
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //frontleft.setTargetPosition((int)(-20 / (4*Math.PI) * 537.7));

        backleft = hardwareMap.get(DcMotor.class, "back left");
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //backleft.setTargetPosition((int)(-20 / (4*Math.PI) * 537.7));

        backright = hardwareMap.get(DcMotor.class, "back right");
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //backright.setTargetPosition((int)(20 / (4*Math.PI) * 537.7));

        frontright = hardwareMap.get(DcMotor.class, "front right");
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //frontright.setTargetPosition((int)(20 / (4*Math.PI) * 537.7));
        
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        
        colorsensor = hardwareMap.get(ColorSensor.class, "color sensor");
        telemetry.addData("Mode", "waiting");
        telemetry.update();
    
    
        waitForStart();
        telemetry.addData("Mode", "running");
        telemetry.update();


        // Start the motors by giving power
        backleft.setPower(-0.25);
        frontright.setPower(0.25);
        backright.setPower(0.25);
        frontleft.setPower(-0.25);

        // Wait while encoders are still counting
        while (opModeIsActive() && frontleft.isBusy()) 
        {
            telemetry.addData("encoder-backleft", backleft.getCurrentPosition() + "  busy=" + backleft.isBusy());
            telemetry.addData("encoder-frontright", frontright.getCurrentPosition() + "  busy=" + frontright.isBusy());
            telemetry.addData("encoder-backright", backright.getCurrentPosition() + "  busy=" + backright.isBusy());
            telemetry.addData("encoder-frontleft", frontleft.getCurrentPosition() + "  busy=" + frontleft.isBusy());
            telemetry.update();
            idle();
        }

        // Cut power from the motors
        backleft.setPower(0.0);
        frontright.setPower(0.0);
        backright.setPower(0.0);
        frontleft.setPower(0.0);

        // Wait 5 seconds just in case something happens
        // resetStartTime();
        while (opModeIsActive() && getRuntime() < 10) 
        {
            telemetry.addData("encoder-backleft", backleft.getCurrentPosition());
            telemetry.addData("encoder-frontright", frontright.getCurrentPosition());
            telemetry.addData("encoder-backright", backright.getCurrentPosition());
            telemetry.addData("encoder-frontleft", frontleft.getCurrentPosition());
            telemetry.update();
            idle();
        }
        
        if (color().equals("green"))
        {
                telemetry.addData("Color", "Green");
                frontleft.setTargetPosition((int)(-20 / (4*Math.PI) * 537.7));
                backleft.setTargetPosition((int)(20 / (4*Math.PI) * 537.7));
                frontright.setTargetPosition((int)(-20 / (4*Math.PI) * 537.7));
                backright.setTargetPosition((int)(20 / (4*Math.PI) * 537.7));
                backleft.setPower(0.25);
                frontright.setPower(-0.25);
                backright.setPower(0.25);
                frontleft.setPower(-0.25);
                telemetry.update();
        }
            /* else if (color().equals("purple"))
            {
                frontleft.setTargetPosition((int)(34 / (4*Math.PI) * 537.7));
                backleft.setTargetPosition((int)(34 / (4*Math.PI) * 537.7));
                frontright.setTargetPosition((int)(-34 / (4*Math.PI) * 537.7));
                backright.setTargetPosition((int)(-34 / (4*Math.PI) * 537.7));
                backleft.setPower(0.25);
                frontright.setPower(-0.25);
                backright.setPower(-0.25);
                frontleft.setPower(0.25);
            }
            else if (color().equals("orange"))
            {
                frontleft.setTargetPosition((int)(34 / (4*Math.PI) * 537.7));
                backleft.setTargetPosition((int)(-34 / (4*Math.PI) * 537.7));
                frontright.setTargetPosition((int)(34 / (4*Math.PI) * 537.7));
                backright.setTargetPosition((int)(-34 / (4*Math.PI) * 537.7));
                backleft.setPower(-0.25);
                frontright.setPower(0.25);
                backright.setPower(-0.25);
                frontleft.setPower(0.25);
                telemetry.addData("Color", "Orange");
                telemetry.update();
            }
            else
            {
                telemetry.addData("Color", "None Detected");
                telemetry.addData("encoder-backleft", backleft.getCurrentPosition() + "  busy=" + backleft.isBusy());
                telemetry.addData("encoder-frontright", frontright.getCurrentPosition() + "  busy=" + frontright.isBusy());
                telemetry.addData("encoder-backright", backright.getCurrentPosition() + "  busy=" + backright.isBusy());
                telemetry.addData("encoder-frontleft", frontleft.getCurrentPosition() + "  busy=" + frontleft.isBusy());
                telemetry.update();
                idle();
            }
        }
        while (True)
        {
            int[] rgb= new int[3];
            rgb[0] = colorsensor.red();
            rgb[1] = colorsensor.green();
            rgb[2] = colorsensor.blue();
            if (color().equals("green"))
            {
                    telemetry.addData("Color", "Green");
                    frontleft.setTargetPosition((int)(-20 / (4*Math.PI) * 537.7));
                    backleft.setTargetPosition((int)(20 / (4*Math.PI) * 537.7));
                    frontright.setTargetPosition((int)(-20 / (4*Math.PI) * 537.7));
                    backright.setTargetPosition((int)(20 / (4*Math.PI) * 537.7));
                    backleft.setPower(0.25);
                    frontright.setPower(-0.25);
                    backright.setPower(0.25);
                    frontleft.setPower(-0.25);
                    telemetry.update();
                    break;
            }
                /*case "purple":
                    frontleft.setTargetPosition((int)(34 / (4*Math.PI) * 537.7));
                    backleft.setTargetPosition((int)(34 / (4*Math.PI) * 537.7));
                    frontright.setTargetPosition((int)(-34 / (4*Math.PI) * 537.7));
                    backright.setTargetPosition((int)(-34 / (4*Math.PI) * 537.7));
                    backleft.setPower(0.25);
                    frontright.setPower(-0.25);
                    backright.setPower(-0.25);
                    frontleft.setPower(0.25);
                    break;
                else if (color().equals("orange"))
                {
                    telemetry.addData("Color", "Orange");
                    frontleft.setTargetPosition((int)(34 / (4*Math.PI) * 537.7));
                    backleft.setTargetPosition((int)(-34 / (4*Math.PI) * 537.7));
                    frontright.setTargetPosition((int)(34 / (4*Math.PI) * 537.7));
                    backright.setTargetPosition((int)(-34 / (4*Math.PI) * 537.7));
                    backleft.setPower(-0.25);
                    frontright.setPower(0.25);
                    backright.setPower(-0.25);
                    frontleft.setPower(0.25);
                    telemetry.update();
                    break;
                }
                else
                {
                    telemetry.addData("Color", ("" + colorsensor.red() + ", " + colorsensor.green() + ", " + colorsensor.blue()));
                    telemetry.update();
                }
            }
            backleft.setPower(0.0);
            frontright.setPower(0.0);
            backright.setPower(0.0);
            frontleft.setPower(0.0);
        }
    

    public String color()
    {
        int orangeCount = 0;
        int greenCount = 0;
        int purpleCount = 0;
        //retrives the contents of the pixel
        //sets the rgb values to their respective variables
        int r = colorsensor.red();
        int g = colorsensor.green();
        int b = colorsensor.blue();
        //sets isGreen if pixel is green
        boolean isGreen = g >= 13 && (b <= (g - 9)) && (r <= (g - 9));
        //sets isPink if pixel is pink
        boolean isPurple = b >= 60 && (r <= (b-3) && g <= r);
        //sets isOrange if pixel is orange
        boolean isOrange = r >= 70 && (g <= (r - 50) && b <= g-22);
                
             
        //checks if the pixel is Orange, Green or Purple and increments their respective counter variables
        if(isOrange){
            orangeCount++;
        }
        else if(isGreen){
            greenCount++;
        }
        else if(isPurple){
            purpleCount++;
        }
        //checks whether there are more orange, green, or purple pixels and returns the highest amount
        if(orangeCount > greenCount && orangeCount > purpleCount){
            return "orange";
        }
        else if(greenCount > orangeCount && greenCount > purpleCount){
            return "green";
        }
        else if(purpleCount > orangeCount && purpleCount > greenCount){
            return "purple";
        }
        else{
            return "Invalid Value";
        }               
    }
 
}*/
