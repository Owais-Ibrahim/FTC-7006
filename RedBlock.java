package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name= "Red Block", group="Sky autonomous")
public class RedBlock extends LinearOpMode {

    HardwarePushbot robot = new HardwarePushbot();


    //Encoder
    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;
    static final double     WHEEL_DIAMETER_INCHES   = 3.858;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     TourqueMotorTicks    = 1440 ;
    static final double     PulleyDiameter   = 1.25;
    static final double     LiftInch         = (TourqueMotorTicks) / (PulleyDiameter * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        GyroCalibirate();
        EncoderReset();

        waitForStart();

        GetBlock1();

    }


    //1st Block Code (Based on vision)
    public void GetBlock1() throws InterruptedException
    {
        EncodedStraightDrive(25,1.0);
        EncodedStrafe(-10,1.0);//Negative for right and positive for left
        GrabBlock(15,0.4);
        robot.Push.setPower(1.0);
        sleep(400);
        EncodedStraightDrive(-20,1.0);
        robot.Push.setPower(1.0);
        sleep(400);
        robot.Grabber.setPosition(1.0);
        robot.Push.setPower(0);
        TurnAbsolute(280,0.5,2);
        EncodedStraightDrive(-85,0.9);
        TurnAbsolute(190,0.5,2);
        EncodedStraightDrive(-5,0.4);
        PlaceBlock();
        GetBlock4();
    }

    public void GetBlock2()throws InterruptedException
    {
        EncodedStraightDrive(25,1.0);
        GrabBlock(15,0.4);
        robot.Push.setPower(1.0);
        sleep(400);
        EncodedStraightDrive(-20,1.0);
        robot.Push.setPower(1.0);
        sleep(400);
        robot.Grabber.setPosition(1.0);
        robot.Push.setPower(0);
        TurnAbsolute(280,0.5,2);
        EncodedStraightDrive(-95,0.9);
        TurnAbsolute(190,0.5,2);
        EncodedStraightDrive(-5,0.4);
        PlaceBlock();
        GetBlock5();
    }

    public void GetBlock3()throws InterruptedException
    {
        EncodedStraightDrive(25,1.0);
        EncodedStrafe(10,1.0);
        GrabBlock(15,0.4);
        robot.Push.setPower(1.0);
        sleep(400);
        EncodedStraightDrive(-20,1.0);
        robot.Push.setPower(1.0);
        sleep(400);
        robot.Grabber.setPosition(1.0);
        TurnAbsolute(280,0.5,2);
        EncodedStraightDrive(-105,0.9);
        TurnAbsolute(190,0.5,2);
        EncodedStraightDrive(-5,0.4);
        PlaceBlock();
        GetBlock6();
    }

    //2nd Block Code (Based on Pattern)
    public void GetBlock4() throws InterruptedException
    {
        EncodedStraightDrive(5,1.0);
        robot.rightFront.setPower(0.5);
        robot.rightBack.setPower(0.5);
        robot.leftFront.setPower(0.5);
        robot.leftBack.setPower(0.5);
        sleep(1000);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        sleep(100);
        TurnAbsolute(270,0.3,2);
        EncodedStraightDrive(105,0.8);
        EncodedStrafe(-6,1.0);
        GrabBlock(5,0.8);
        robot.Push.setPower(1.0);
        sleep(400);
        robot.Grabber.setPosition(1.0);
        robot.Push.setPower(0);
        EncodedStrafe(8,1.0);
        TurnAbsolute(270,0.3,2);
        EncodedStraightDrive(-105,0.9);
        TurnAbsolute(190,0.5,2);
        EncodedStraightDrive(-5,0.4);
        PlaceBlock();
        Park();
    }

    public void GetBlock5()throws InterruptedException
    {
        EncodedStraightDrive(5,1.0);
        robot.rightFront.setPower(0.5);
        robot.rightBack.setPower(0.5);
        robot.leftFront.setPower(0.5);
        robot.leftBack.setPower(0.5);
        sleep(1000);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        sleep(100);
        TurnAbsolute(270,0.3,2);
        EncodedStraightDrive(115,0.9);
        EncodedStrafe(-6,1.0);
        GrabBlock(5,0.8);
        robot.Push.setPower(1.0);
        sleep(500);
        robot.Grabber.setPosition(1.0);
        robot.Push.setPower(0);
        EncodedStrafe(8,1.0);
        TurnAbsolute(270,0.3,2);
        EncodedStraightDrive(-115,0.9);
        TurnAbsolute(190,0.5,2);
        EncodedStraightDrive(-5,0.4);
        PlaceBlock();
        Park();
    }

    public void GetBlock6()throws InterruptedException
    {
        EncodedStraightDrive(5,1.0);
        robot.rightFront.setPower(0.5);
        robot.rightBack.setPower(0.5);
        robot.leftFront.setPower(0.5);
        robot.leftBack.setPower(0.5);
        sleep(1000);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        sleep(100);
        TurnAbsolute(270,0.4,2);
        EncodedStraightDrive(125,0.9);
        EncodedStrafe(-6,1.0);
        GrabBlock(5,0.8);
        robot.Push.setPower(1.0);
        sleep(500);
        robot.Grabber.setPosition(1.0);
        robot.Push.setPower(0);
        EncodedStrafe(8,1.0);
        EncodedStraightDrive(-125,0.9);
        TurnAbsolute(190,0.5,2);
        EncodedStraightDrive(-5,0.4);
        PlaceBlock();
        Park();
    }


    public void Park() throws InterruptedException
    {
        EncodedStraightDrive(5,1.0);
        robot.rightFront.setPower(0.5);
        robot.rightBack.setPower(0.5);
        robot.leftFront.setPower(0.5);
        robot.leftBack.setPower(0.5);
        sleep(1000);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        sleep(100);
        TurnAbsolute(270,0.3,2);
        EncodedStraightDrive(60,1.0);
    }
    public void PlaceBlock() throws InterruptedException
    {

        LiftUp(8,1.0);
        sleep(100);
        robot.GrabMove.setPosition(1.0);
        sleep(300);
        //LiftDown(9,1.0);
        //sleep(100);
        robot.Grabber.setPosition(0.5);
        sleep(100);
        robot.GrabMove.setPosition(0);
        LiftDown(8,1.0);

    }

    //Encoder

    public void LiftUp (double inches, double speed) throws InterruptedException
    {
        int RotationsNeeded= (int)(inches*(LiftInch));

        robot.Lift.setTargetPosition((-RotationsNeeded)+(robot.Lift.getCurrentPosition()));
        // Turn On RUN_TO_POSITION
        robot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Lift.setPower(speed);
        while (robot.Lift.isBusy())
        {
            // Display it for the driver.
            telemetry.addData("Lift Up:", robot.Lift.getCurrentPosition());
            telemetry.update();
        }

        waitOneFullHardwareCycle();

        // Stop all motion;
        robot.Lift.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void LiftDown (double inches, double speed) throws InterruptedException
    {
        int RotationsNeeded= (int)(inches*(LiftInch));

        robot.Lift.setTargetPosition((RotationsNeeded)+(robot.Lift.getCurrentPosition()));
        // Turn On RUN_TO_POSITION
        robot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Lift.setPower(speed);
        while (robot.Lift.isBusy())
        {
            robot.Push.setPower(-0.6);

            // Display it for the driver.
            telemetry.addData("Lift Up:", robot.Lift.getCurrentPosition());
            telemetry.update();
        }

        waitOneFullHardwareCycle();

        // Stop all motion;
        robot.Lift.setPower(0);
        robot.Push.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void GrabBlock(double inches, double speed) throws InterruptedException //Make negative distance if want to forward or backward depending on motors
    {
        int RotationsNeeded= (int)(inches*(COUNTS_PER_INCH));

        robot.rightFront.setTargetPosition((-RotationsNeeded)+(robot.rightFront.getCurrentPosition()));
        robot.rightBack.setTargetPosition((-RotationsNeeded)+(robot.rightBack.getCurrentPosition()));
        robot.leftFront.setTargetPosition((RotationsNeeded)+(robot.leftFront.getCurrentPosition()));
        robot.leftBack.setTargetPosition((RotationsNeeded)+(robot.leftBack.getCurrentPosition()));

        telemetry.addData("RightFront:", RotationsNeeded);
        telemetry.addData("RightBack:", RotationsNeeded);
        telemetry.addData("LeftFront:", RotationsNeeded);
        telemetry.addData("LeftBack:", RotationsNeeded);

        telemetry.update();



        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //sleep(1000);
        robot.rightFront.setPower(speed);
        robot.rightBack.setPower(speed);
        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(speed);
        //sleep(100);

        while (opModeIsActive() && (robot.rightFront.isBusy() && robot.leftFront.isBusy())
                && (robot.rightBack.isBusy() && robot.leftBack.isBusy()))
        {
            robot.rightIntake.setPower(-0.8);
            robot.leftIntake.setPower(0.8);

            // Display it for the driver.
            telemetry.addData("Running:", robot.rightFront.getCurrentPosition());
            telemetry.addData("Left:", robot.leftFront.getCurrentPosition());
            telemetry.update();

        }

        waitOneFullHardwareCycle();


        // Stop all motion;
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);

        robot.rightIntake.setPower(0);
        robot.leftIntake.setPower(0);
        // Turn off RUN_TO_POSITION
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void EncodedStraightDrive(double inches, double speed) throws InterruptedException //Make negative distance if want to forward or backward depending on motors
    {
        int RotationsNeeded= (int)(inches*(COUNTS_PER_INCH));

        robot.rightFront.setTargetPosition((-RotationsNeeded)+(robot.rightFront.getCurrentPosition()));
        robot.rightBack.setTargetPosition((-RotationsNeeded)+(robot.rightBack.getCurrentPosition()));
        robot.leftFront.setTargetPosition((RotationsNeeded)+(robot.leftFront.getCurrentPosition()));
        robot.leftBack.setTargetPosition((RotationsNeeded)+(robot.leftBack.getCurrentPosition()));

        telemetry.addData("RightFront:", RotationsNeeded);
        telemetry.addData("RightBack:", RotationsNeeded);
        telemetry.addData("LeftFront:", RotationsNeeded);
        telemetry.addData("LeftBack:", RotationsNeeded);

        telemetry.update();



        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //sleep(1000);
        robot.rightFront.setPower(speed);
        robot.rightBack.setPower(speed);
        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(speed);
        //sleep(100);

        while (opModeIsActive() && (robot.rightFront.isBusy() && robot.leftFront.isBusy())
                && (robot.rightBack.isBusy() && robot.leftBack.isBusy()))
        {

        }

        waitOneFullHardwareCycle();


        // Stop all motion;
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    //Negative for right and positive for left
    public void EncodedStrafe(double inches, double speed) throws InterruptedException //Make negative if want to go right but positive if want to go left
    {
        int RotationsNeeded= (int)(inches*(COUNTS_PER_INCH));


        robot.rightFront.setTargetPosition((-RotationsNeeded)+(robot.rightFront.getCurrentPosition()));
        robot.rightBack.setTargetPosition((RotationsNeeded)+(robot.rightBack.getCurrentPosition()));
        robot.leftFront.setTargetPosition((-RotationsNeeded)+(robot.leftFront.getCurrentPosition()));
        robot.leftBack.setTargetPosition((RotationsNeeded)+(robot.leftBack.getCurrentPosition()));

        // Turn On RUN_TO_POSITION
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.rightFront.setPower(speed);
        robot.rightBack.setPower(speed);
        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(speed);

        while (opModeIsActive() && (robot.rightFront.isBusy() && robot.leftFront.isBusy())
                && (robot.rightBack.isBusy() && robot.leftBack.isBusy()))
        {



            // Display it for the driver.
            telemetry.addData("Right:", robot.rightFront.getCurrentPosition());
            telemetry.update();

        }

        waitOneFullHardwareCycle();


        // Stop all motion;
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void EncoderReset()throws InterruptedException
    {
        telemetry.addLine("Status Resetting Encoders");
        telemetry.update();


        //robot.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Right:", "at %2d", robot.rightFront.getCurrentPosition());
        telemetry.update();
        telemetry.addData("Left:", "at %2d", robot.leftFront.getCurrentPosition());
        telemetry.update();


    }


    //Gyro
    public void GyroCalibirate() throws InterruptedException
    {
        robot.gyro.calibrate();

        while (robot.gyro.isCalibrating())
        {
            telemetry.addLine("Calibrating");
            telemetry.update();
        }

        telemetry.addLine("Robot Done Calibrating");
        telemetry.update();
    }

    public void TurnAbsolute(int target,double turnspeed, int threshold) throws InterruptedException
    {
        int Heading= robot.gyro.getHeading();
        if (Heading>180)
        {
            Heading= robot.gyro.getHeading()-360;
        }

        while (Math.abs(Heading-target) > threshold)
        {
            if (Heading>180)
            {
                Heading= robot.gyro.getHeading()-360;
            }

            telemetry.addData("CurrentHeading:", Heading);
            telemetry.update();

            if (Heading>target)
            {
                //CW
                robot.rightFront.setPower(turnspeed);
                robot.rightBack.setPower(turnspeed);
                robot.leftFront.setPower(turnspeed);
                robot.leftBack.setPower(turnspeed);

                telemetry.addData("Heading>Target",Heading );
                telemetry.update();

            }
            if (Heading<target)
            {
                //CCW
                robot.rightFront.setPower(-turnspeed);
                robot.rightBack.setPower(-turnspeed);
                robot.leftFront.setPower(-turnspeed);
                robot.leftBack.setPower(-turnspeed);

                telemetry.addData("Heading<Target", Heading );
                telemetry.update();
            }


            Heading=robot.gyro.getHeading();
        }



        waitOneFullHardwareCycle();

        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);

        telemetry.addData("CurrentHeading:", robot.gyro.getHeading());
        telemetry.update();
    }

     /*
    Foundation

    public void Foundation() throws InterruptedException
    {
        robot.GrabFoundationRight.setPosition(0.5);
        robot.GrabFoundationLeft.setPosition(0.5);
        sleep(1000);
        EncodedStraightDrive(20,0.3);
        robot.rightFront.setPower(0.4);
        robot.rightBack.setPower(0.4);
        robot.leftFront.setPower(0.4);
        robot.leftBack.setPower(0.4);
        sleep(1000);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        sleep(100);
        TurnAbsolute(270,0.3,3);
    }

     */

}
