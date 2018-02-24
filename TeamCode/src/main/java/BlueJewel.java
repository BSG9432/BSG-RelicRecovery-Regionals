
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AutoTransition;

@Autonomous(name="Blue Jewel", group="y")
public class BlueJewel extends LinearOpMode {

    public DcMotor rightM;
    public DcMotor leftM;
    public DcMotor rightC;
    public DcMotor leftC;
    public DcMotor liftT;
    public DcMotor liftB;
    public DcMotor rotate;
    public DcMotor relicLift;
    public Servo rightH;
    public ColorSensor rightS;

    static final double     COUNTS_PER_MOTOR_REV    = 1120;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = .4;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    private ElapsedTime runtime = new ElapsedTime();

    double r = 49.5;
    double l = 34.5;
    double c = 42;

    @Override
    public void runOpMode() {

        rightM = hardwareMap.dcMotor.get("rightM");
        leftM = hardwareMap.dcMotor.get("leftM");
        rightC = hardwareMap.dcMotor.get("rightC");
        leftC = hardwareMap.dcMotor.get("leftC");
        liftT = hardwareMap.dcMotor.get("liftT");
        liftB = hardwareMap.dcMotor.get("liftB");
        rotate = hardwareMap.dcMotor.get("rotate");
        relicLift = hardwareMap.dcMotor.get("relicLift");
        rightH = hardwareMap.servo.get("rightH");
        rightS = hardwareMap.colorSensor.get("rightS");

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        boolean bPrevState = false;
        boolean bCurrState = false;
        boolean bLedOn = true;

        telemetry.update();

        AutoTransition.transitionOnStop(this, "TeleOp");

        waitForStart();

        sleep(1000);

        telemetry.update();

        rightH.setPosition(.2);
        sleep(500);
        rightH.setPosition(.1);
        sleep(500);        //arm down

        sleep(1000);

        telemetry.update();

        if (rightS.blue() < rightS.red()) {
            encoderDrive(.5,-1,-4,10);
            rightH.setPosition(.95);    //arm up
            encoderDrive(.5,1,4,10);

        } else {
            encoderDrive(.5,1,4,10);
            rightH.setPosition(.95);   //arm up
            encoderDrive(.5,-1,-4,10);
        }
    }
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftM.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightM.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftM.setTargetPosition(newLeftTarget);
            rightM.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftM.setPower(Math.abs(speed));
            rightM.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftM.isBusy() && rightM.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftM.getCurrentPosition(),
                        rightM.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftM.setPower(0);
            rightM.setPower(0);

            // Turn off RUN_TO_POSITION
            leftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }
}