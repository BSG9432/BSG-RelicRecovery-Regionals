/*
import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.DonaldHardware;
import org.firstinspires.ftc.teamcode.AutoTransition;


@Autonomous(name = "JETSTUFF", group = "TEST")
public class JetEncoders extends LinearOpMode
{
    DonaldHardware robot = new DonaldHardware();
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: AndyMark dMotor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.66 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;
    static final double     TURN_SPEED              = 0.1;
    public ColorSensor rightS;


    @Override
    public void runOpMode() {

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        boolean bPrevState = false;
        boolean bCurrState = false;
        boolean bLedOn = true;

        telemetry.update();

        AutoTransition.transitionOnStop(this, "TeleOp");
        robot.init(hardwareMap);

        rightS = hardwareMap.get(ColorSensor.class, "rightS");
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftM.getCurrentPosition(),
                robot.rightM.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  4,  4, 1.0);  // S1: Forward 4 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED,   12, -12, 1.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -24, -24, 1.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        //WE DONT USE THIS

      //  robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
      //  robot.rightClaw.setPosition(0.0);
      //  sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     *//*
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftM.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightM.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftM.setTargetPosition(newLeftTarget);
            robot.rightM.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftM.setPower(Math.abs(.1));
            robot.rightM.setPower(Math.abs(.1));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftM.isBusy() && robot.rightM.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftM.getCurrentPosition(),
                        robot.rightM.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftM.setPower(0);
            robot.rightM.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    //function for the RED SIDE
    public void detectJewel(double timeoutS){
        if(rightS.blue() < rightS.red()) {
            encoderDrive(TURN_SPEED, -2, 2, 2.0);
            encoderDrive(TURN_SPEED, 2,-2,2.0);
        }
        else {
            encoderDrive(TURN_SPEED, 2, -2,2.0);
            encoderDrive(TURN_SPEED, -2,2, 2.0);
        }

    }
}
*/