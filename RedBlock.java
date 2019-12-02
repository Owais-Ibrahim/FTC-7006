import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwarePushbot;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="RedBlock", group="Pushbot")
public class RedBlock extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();
    boolean parktop;


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
      
        waitForStart();
            
          
        StrafeRight(0.45,1750); //Close to block
        sleep(1000);
        Drive(.5,475); //3rd block
        sleep(100);
        robot.arm.setPosition(1.0); //Grab
        sleep(1000); 
        StrafeLeft(0.4,900); //Left with block
        sleep(1000);
        Drive(-0.4,2900); //Drive under bridge
        sleep(1000);
        robot.arm.setPosition(0); //Servo up
        sleep(1000);
        Drive(0.4,2000); //Drive for 1st
        sleep(1000);
        StrafeRight(0.45,1300); //Close to 1st
        sleep(1000);
        robot.arm.setPosition(1.0);//Grab
        sleep(1000);
        StrafeLeft(0.5,1400); //Move left
        sleep(1000);
        Drive(-0.4,1850); //Goes under bridge
        sleep(1000);
        robot.arm.setPosition(0); //Move arm up
        sleep(1000);
        Drive(0.4,800); //Drive under bridge
        sleep(1000);
        StrafeRight(0.55,600); //Clear up space
        sleep(1000);

    }
    public void StrafeLeft (double strafe, int sleep)
    {
        robot.rightFront.setPower(strafe);
        robot.rightBack.setPower(-strafe);
        robot.leftFront.setPower(strafe);
        robot.leftBack.setPower(-strafe);

        sleep(sleep);

        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);

        sleep (100);
    }

    public void StrafeRight (double strafe, int sleep)
    {
        robot.rightFront.setPower(-strafe);
        robot.rightBack.setPower(strafe);
        robot.leftFront.setPower(-strafe);
        robot.leftBack.setPower(strafe);

        sleep(sleep);

        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);

        sleep (100);
    }

    public void Drive (double drive, int sleep)
    {
        robot.rightFront.setPower(drive);
        robot.rightBack.setPower(drive);
        robot.leftFront.setPower(-drive);
        robot.leftBack.setPower(-drive);

        sleep(sleep);

        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);

        sleep (100);
    }

}