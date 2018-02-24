import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.AutoTransition;

import static org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark.*;

@Autonomous(name="BlueVu", group = "z")
@Disabled
public class BlueVu extends LinearOpMode {

    public DcMotor rightM;
    public DcMotor leftM;
    public DcMotor rightC;
    public DcMotor leftC;
    public DcMotor liftT;
    public DcMotor liftB;
    public Servo rightH;
    public ColorSensor rightS;
    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = .66;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    // static final double ticks = 560;
    double r = 41.5;
    double l = 26.5;
    double c = 34;

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    private double TICKS_PER_IN = (1120*.66) / (4*Math.PI);

    public void encoderDrive(double speed,
                             double leftInches, double rightInches) {
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
            leftM.setPower(Math.abs(speed));
            rightM.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
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

            //  sleep(250);   // optional pause after each move
        }
    }

    public void driveDistance(double inches, double leftPower, double rightPower) {

        leftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

       /*create two integers that use the current motor
       position, the inches given, and the calculation from motor
       tics to inches in order to tell the motor how far to go for each side*/
        int leftTickGoal = leftM.getCurrentPosition() + (int) (TICKS_PER_IN * inches);
        int rightTickGoal = rightM.getCurrentPosition() + (int) (TICKS_PER_IN * inches);

       /*sets the motors to the new goal created above*/
        leftM.setTargetPosition(leftTickGoal);
        rightM.setTargetPosition(rightTickGoal);

        //tells motors to go to the position set above
        leftM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

       /*tells the motors to continue moving until the new goal is acheived*/
       // while (leftM.isBusy() && rightM.isBusy()) {

            //sets left and right motors to power given
            rightM.setPower(rightPower/2);//posss -1
            leftM.setPower(leftPower/2);//poss +1

            telemetry.addData("Left Encoder: ", leftM.getCurrentPosition());
            telemetry.addData("Right Encoder: ", rightM.getCurrentPosition());
            updateTelemetry(telemetry);
            telemetry.update();
        }
    //}



    @Override
    public void runOpMode() {

        rightM = hardwareMap.dcMotor.get("rightM");
        leftM = hardwareMap.dcMotor.get("leftM");
        rightC = hardwareMap.dcMotor.get("rightC");
        leftC = hardwareMap.dcMotor.get("leftC");
        liftT = hardwareMap.dcMotor.get("liftT");
        liftB = hardwareMap.dcMotor.get("liftB");
        rightH = hardwareMap.servo.get("rightH");
        rightS = hardwareMap.colorSensor.get("rightS");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        //https://developer.vuforia.com/license-manager

        parameters.vuforiaLicenseKey = "AcaIHID/////AAAAGfxtLCdtek2StO17ue0Qv58vOtoKhEUMbPtKsKEn2j6O+r8N8aHlIv8rlPkvXTUfpjbpjashfAtx/NZhQJemzLtk9BjxwlW5VGYaUbOpiLxz8L0yxh8owF8XbCf79eLaUmD7J8BpMgyNKPijZJWl2r7uwc+Mmum8S0cnwliFN2/Pt8y6aYd0eEppco4tMn/WrkQ+GspPLUZ7CGRYh2goHG14ZSuVOXEDx7poPvGgYm9A5pCzuSNSE1dl0uppOAJaw307cZ6YIKX3CZgw92+XquVm0fvfHrLcR1Cj09R56vZKoEZH4S+uWaTofEwlGK7QRp5Kz2CNn8S8W5E80COVniyNPjIRi5WWjl6qDSY6nyrH";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        AutoTransition.transitionOnStop(this, "TeleOp");

        waitForStart();

        sleep(1000);

        telemetry.update();

        rightH.setPosition(.9);
        rightH.setPosition(1);
        sleep(1000);        //arm down

        rightC.setPower(-.1);
        leftC.setPower(.1);
        sleep(500);        //clamp
        rightC.setPower(-.1);
        leftC.setPower(.1);

        telemetry.update();

        if (rightS.blue() < rightS.red()) {

            telemetry.update();

            rightM.setPower(-.4);
            sleep(750);     //back up

            rightM.setPower(0);
            sleep(500);     //pause

            rightH.setPosition(.1);    //arm up

            rightM.setPower(.4);
            sleep(750);     //forward

        } else {

            telemetry.update();

            rightM.setPower(.4);
            sleep(750);     //drive forward

            rightM.setPower(0);
            sleep(500);     //pause

            rightH.setPosition(.1);   //arm up

            rightM.setPower(-.4);
            sleep(750);     //back up

        }

        relicTrackables.activate();

        while (opModeIsActive()) {

            RelicRecoveryVuMark vuMark = from(relicTemplate);
            if (vuMark != UNKNOWN) {

                telemetry.addData("VuMark", "%s visible", vuMark);

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }

                telemetry.update();
            } else {
                telemetry.addData("VuMark", "not visible");
            }

            if (vuMark == RelicRecoveryVuMark.LEFT) {

                driveDistance(l,.25,.25);

            }
            else if (vuMark == RelicRecoveryVuMark.CENTER) {

                driveDistance(c,.25,.25);

            }
            else if (vuMark == RelicRecoveryVuMark.RIGHT) {

                driveDistance(r,.25,.25);

            }
            else {

                driveDistance(0,0,0);

            }

                relicTrackables.deactivate();

            telemetry.update();
        }
        telemetry.update();
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}