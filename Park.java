package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;



@Autonomous(name= "Park", group="Sky autonomous")
//@Disabled
public class Park extends LinearOpMode {

    HardwarePushbot robot = new HardwarePushbot();



    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);


        waitForStart();
        sleep(25000);
       double speed=0.4;

        robot.rightFront.setPower(-speed);
        robot.rightBack.setPower(-speed);
        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(speed);

        sleep(2000);

        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        sleep(200);

       /*
        robot.Measure.setPower(1.0);
        sleep(3000);
        robot.Measure.setPower(0);
        sleep(100);

        */
    }


}
