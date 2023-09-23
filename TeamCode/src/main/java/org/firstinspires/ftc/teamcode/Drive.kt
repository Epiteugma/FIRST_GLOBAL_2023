package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit
import kotlin.concurrent.thread

@TeleOp(name = "Drive (FGC 2023)", group = "FGC 2023")
class Drive: LinearOpMode() {
    // Control variables
    private val DRIVE_MLT = 1.0
    private val SLIDE_MLT = 1.0
    private val HOLD_POWER = 1.0

    private lateinit var left: DcMotor
    private lateinit var right: DcMotor
    private lateinit var leftSlide: DcMotor
    private lateinit var rightSlide: DcMotor

    override fun runOpMode() {
        // Hardware Initialization
        left = hardwareMap.get(DcMotor::class.java, "left")
        right = hardwareMap.get(DcMotor::class.java, "right")
        leftSlide = hardwareMap.get(DcMotor::class.java, "leftSlide")
        rightSlide = hardwareMap.get(DcMotor::class.java, "rightSlide")

        // Brake drivetrain when control released
        left.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        right.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        var initTime = System.currentTimeMillis()
        var lastDotsIncrement = System.currentTimeMillis()
        var dots = 0

        // Waiting for start telemetry
        while(opModeInInit()) {
            telemetry.addData("Waiting for start" + ".".repeat(dots), "")
            telemetry.update()

            if(lastDotsIncrement + 500 > System.currentTimeMillis()) {
                lastDotsIncrement = System.currentTimeMillis()
                dots = (dots + 1) % 4
            }
        }

        var startTime = System.currentTimeMillis()

        // Driver 1 Thread
        thread(name = "DRIVER1", start = true) {
            while (opModeIsActive()) {
                left.power = (-gamepad1.left_stick_y - gamepad1.left_stick_x) * DRIVE_MLT
                right.power = (-gamepad1.left_stick_y + gamepad1.left_stick_x) * DRIVE_MLT
            }
        }

        // Driver 2 Thread
        thread(name = "DRIVER2", start = true) {
            while (opModeIsActive()) {
                if(gamepad2.left_stick_y != 0.0f) for (motor in listOf(leftSlide, rightSlide)) {
                    motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                    motor.power = gamepad2.left_stick_y * SLIDE_MLT
                } else for (motor in listOf(leftSlide, rightSlide)) {
                    motor.targetPosition = motor.currentPosition
                    motor.mode = DcMotor.RunMode.RUN_TO_POSITION
                    motor.power = HOLD_POWER
                }
            }
        }

        // Telemetry in main thread
        while (opModeIsActive()) {
            telemetry.addData("--- DRIVETRAIN ---", "")
            telemetry.addData("Left Power", left.power)
            telemetry.addData("Right Power", right.power)
            telemetry.addData("", "")

            telemetry.addData("--- SLIDES ---", "")
            telemetry.addData("Mode", leftSlide.mode.toString())
            if(leftSlide.mode == DcMotor.RunMode.RUN_TO_POSITION) {
                telemetry.addData("Left target", leftSlide.targetPosition)
                telemetry.addData("Right target", rightSlide.targetPosition)
            } else telemetry.addData("Power", leftSlide.power)
            telemetry.addData("", "")

            telemetry.addData("--- HEALTH ---", "")
            telemetry.addData("Runtime since INIT", String.format("%.2fs", (System.currentTimeMillis() - initTime) / 1000))
            telemetry.addData("Runtime since START", String.format("%.2fs", (System.currentTimeMillis() - startTime) / 1000))

            val hubs = hardwareMap.getAll(LynxModule::class.java)
            telemetry.addData("Hubs detected", hubs.size)
            for(i in 0..hubs.size) {
                telemetry.addData("HUB$i voltage", String.format("%.2fV", hubs[i].getInputVoltage(VoltageUnit.VOLTS)))
                telemetry.addData("HUB$i current", String.format("%.2fA", hubs[i].getCurrent(CurrentUnit.AMPS)))
            }

            telemetry.update()
        }
    }
}