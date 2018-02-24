import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark.CENTER;
import static org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark.LEFT;
import static org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark.RIGHT;
import static org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark.UNKNOWN;

/**
 * Created by 299970 on 2/22/2018.
 */

@Autonomous(name = "Blue 2 ")
public class RGNblue2 extends LinearOpMode {

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

    double r = 56.5;
    double l = 34.5;
    double c = 43.5;

        public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {

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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AcaIHID/////AAAAGfxtLCdtek2StO17ue0Qv58vOtoKhEUMbPtKsKEn2j6O+r8N8aHlIv8rlPkvXTUfpjbpjashfAtx/NZhQJemzLtk9BjxwlW5VGYaUbOpiLxz8L0yxh8owF8XbCf79eLaUmD7J8BpMgyNKPijZJWl2r7uwc+Mmum8S0cnwliFN2/Pt8y6aYd0eEppco4tMn/WrkQ+GspPLUZ7CGRYh2goHG14ZSuVOXEDx7poPvGgYm9A5pCzuSNSE1dl0uppOAJaw307cZ6YIKX3CZgw92+XquVm0fvfHrLcR1Cj09R56vZKoEZH4S+uWaTofEwlGK7QRp5Kz2CNn8S8W5E80COVniyNPjIRi5WWjl6qDSY6nyrH";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();                                                                             //START

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
        rightM.getCurrentPosition(),
        leftM.getCurrentPosition());
        telemetry.update();

        relicTrackables.activate();

        while (opModeIsActive()) {

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }                                                                                       //START SWITCH CASE HERE

            if (vuMark == LEFT){
                relicTrackables.deactivate();
                encoderDrive(.25,l,-l,10);
                encoderDrive(.25,-14,-14,10);
                encoderDrive(.25,-10,10,10);
            }
            else if (vuMark == CENTER){
                relicTrackables.deactivate();
                encoderDrive(.25,c,-c,10);
                encoderDrive(.25,-14,-14,10);
                encoderDrive(.25,-10,10,10);
            }
            else if (vuMark == RIGHT) {
                relicTrackables.deactivate();
                encoderDrive(.25,r,-r,10);
                encoderDrive(.25,-14,-14,10);
                encoderDrive(.25,-10,10,10);
            }
            else {
                encoderDrive(0,0,0,0);
            }

            if (vuMark != UNKNOWN){
                relicTrackables.deactivate();
            }
            telemetry.update();
        }
        telemetry.update();
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
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
