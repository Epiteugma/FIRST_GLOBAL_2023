package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit
import kotlin.concurrent.thread

@TeleOp(name = "Drive (FGC 2023)", group = "FGC 2023")
class Drive: LinearOpMode() {
    // Control variables
    private val DRIVE_MLT = 1.0
    private val SLIDE_MLT = 1.0
    private val HOLD_POWER = 1.0

    private val FILTER_UPWARDS = arrayOf(0.8, 0.0)
    private val FILTER_DOWNWARDS = arrayOf(0.09, 0.67)
    private val FILTER_CENTER = arrayOf(0.42, 0.34)

    private val STORAGE_CLOSED = arrayOf(0.22, 1.0)
    private val STORAGE_OPEN = arrayOf(0.6, 0.62)

    // Hardware Devices
    private lateinit var left: DcMotor
    private lateinit var right: DcMotor
    private lateinit var leftLift: DcMotor
    private lateinit var rightLift: DcMotor

    private lateinit var filterServoLeft: Servo
    private lateinit var filterServoRight: Servo
    private lateinit var storageServoLeft: Servo
    private lateinit var storageServoRight: Servo

    // Servo State Enums
    private enum class FilterState {
        NONE,
        UPWARDS,
        DOWNWARDS,
        CENTER;
    }

    private enum class StorageState {
        NONE,
        OPEN,
        CLOSED;
    }

    // Servo State Variables
    private var filterState = FilterState.NONE
    private var lastFilterState = FilterState.NONE
    private var storageState = StorageState.NONE
    private var lastStorageState = StorageState.NONE

    override fun runOpMode() {
        // Hardware Initialization
        left = hardwareMap.get(DcMotor::class.java, "left")
        right = hardwareMap.get(DcMotor::class.java, "right")
        leftLift = hardwareMap.get(DcMotor::class.java, "leftLift")
        rightLift = hardwareMap.get(DcMotor::class.java, "rightLift")

        filterServoLeft = hardwareMap.get(Servo::class.java, "filterLeft")
        filterServoRight = hardwareMap.get(Servo::class.java, "filterRight")
        storageServoLeft = hardwareMap.get(Servo::class.java, "storageLeft")
        storageServoRight = hardwareMap.get(Servo::class.java, "storageRight")

        // Brake drivetrain when control released
        left.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        right.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        val initTime = System.currentTimeMillis()
        waitForStart()
        val startTime = System.currentTimeMillis()

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
                if(gamepad2.left_stick_y != 0.0f) for (motor in listOf(leftLift, rightLift)) {
                    motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                    motor.power = gamepad2.left_stick_y * SLIDE_MLT
                } else for (motor in listOf(leftLift, rightLift)) {
                    motor.targetPosition = motor.currentPosition
                    motor.mode = DcMotor.RunMode.RUN_TO_POSITION
                    motor.power = HOLD_POWER
                }

                if(lastFilterState != filterState) {
                    when(filterState) {
                        FilterState.UPWARDS -> {
                            filterServoLeft.position = FILTER_UPWARDS[0]
                            filterServoRight.position = FILTER_UPWARDS[1]
                        }
                        FilterState.DOWNWARDS -> {
                            filterServoLeft.position = FILTER_DOWNWARDS[0]
                            filterServoRight.position = FILTER_DOWNWARDS[1]
                        }
                        FilterState.CENTER -> {
                            filterServoLeft.position = FILTER_CENTER[0]
                            filterServoRight.position = FILTER_CENTER[1]
                        }
                        else -> {}
                    }

                    lastFilterState = filterState
                }

                if(lastStorageState != storageState) {
                    when(storageState) {
                        StorageState.OPEN -> {
                            storageServoLeft.position = STORAGE_OPEN[0]
                            storageServoRight.position = STORAGE_OPEN[1]
                        }
                        StorageState.CLOSED -> {
                            storageServoLeft.position = STORAGE_CLOSED[0]
                            storageServoRight.position = STORAGE_CLOSED[1]
                        }
                        else -> {}
                    }

                    lastStorageState = storageState
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
            telemetry.addData("Mode", leftLift.mode)
            if(leftLift.mode == DcMotor.RunMode.RUN_TO_POSITION) {
                telemetry.addData("Left target", leftLift.targetPosition)
                telemetry.addData("Right target", rightLift.targetPosition)
            } else telemetry.addData("Power", leftLift.power)
            telemetry.addData("", "")

            telemetry.addData("--- SERVOS ---", "")
            telemetry.addData("Storage state", storageState)
            telemetry.addData("Left Storage Servo", storageServoLeft.position)
            telemetry.addData("Right Storage Servo", storageServoRight.position)
            telemetry.addData("Filter state", filterState)
            telemetry.addData("Left Filter Servo", filterServoLeft.position)
            telemetry.addData("Right Filter Servo", filterServoRight.position)
            telemetry.addData("", "")

            telemetry.addData("--- HEALTH ---", "")
            telemetry.addData("Runtime since INIT", String.format("%.2fs", (System.currentTimeMillis() - initTime) / 1000f))
            telemetry.addData("Runtime since START", String.format("%.2fs", (System.currentTimeMillis() - startTime) / 1000f))

            val hubs = hardwareMap.getAll(LynxModule::class.java)
            telemetry.addData("Hubs detected", hubs.size)
            for(i in 0..<hubs.size) {
                telemetry.addData("HUB$i voltage", String.format("%.2fV", hubs[i].getInputVoltage(VoltageUnit.VOLTS)))
                telemetry.addData("HUB$i current", String.format("%.2fA", hubs[i].getCurrent(CurrentUnit.AMPS)))
            }

            telemetry.update()
        }
    }
}