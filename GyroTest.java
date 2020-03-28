package org.firstinspires.ftc.teamcode.TESTING;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwarePushbot;

@Autonomous(name="GyroTest", group="Pushbot")
@Disabled
public class GyroTest extends LinearOpMode
{
    HardwarePushbot robot = new HardwarePushbot();

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.init(hardwareMap);
        GyroCalibirate();

        waitForStart();

        TurnAbsolute(90,0.2,2);
        sleep(100);

    }

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
                robot.rightFront.setPower(-turnspeed);
                robot.rightBack.setPower(-turnspeed);
                robot.leftFront.setPower(-turnspeed);
                robot.leftBack.setPower(-turnspeed);

                telemetry.addData("Heading>Target",Heading );
                telemetry.update();

            }
            if (Heading<target)
            {
                //CCW
                robot.rightFront.setPower(turnspeed);
                robot.rightBack.setPower(turnspeed);
                robot.leftFront.setPower(turnspeed);
                robot.leftBack.setPower(turnspeed);

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

}
