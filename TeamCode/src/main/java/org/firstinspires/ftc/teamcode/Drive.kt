package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit
import kotlin.concurrent.thread

@TeleOp(name = "Drive (FGC 2023)", group = "FGC 2023")
class Drive: LinearOpMode() {
    // Control variables
    private val DRIVE_MODE = DriveMode.JOYSTICK
    private val COLLECTOR_STALL_THRESHOLD = 1000
    private val COLLECTOR_STALL_RELEASE_TIME = 500

    private val DRIVE_MLT = 1.0
    private val LIFT_MLT = 1.0
    private val COLLECTOR_MLT = 1.0
    private val HOLD_POWER = 1.0

    private val DRIVE_TYPE = MotorType.HD_HEX
    private val LIFT_TYPE = MotorType.HD_HEX
    private val COLLECTOR_TYPE = MotorType.HD_HEX
    private val HOOK_TYPE = MotorType.HD_HEX

    private val FILTER_DOWNWARDS = arrayOf(0.447, 0.637)
    private val FILTER_CENTER = arrayOf(0.766, 0.322)

    private val STORAGE_CLOSED = arrayOf(0.22, 1.0)
    private val STORAGE_OPEN = arrayOf(0.6, 0.62)

    // Hardware Devices
    private lateinit var right: DcMotorEx
    private lateinit var left: DcMotorEx
    private lateinit var rightLift: DcMotorEx
    private lateinit var leftLift: DcMotorEx
    private lateinit var collector: DcMotorEx
    private lateinit var hook: DcMotorEx

    private lateinit var filterServoRight: Servo
    private lateinit var filterServoLeft: Servo
    private lateinit var storageServoRight: Servo
    private lateinit var storageServoLeft: Servo

    // Collector toggle
    private var collectorOn = true
    private var collectorLock = false

    // Motor constants
    private val driveTicksPerSec = DRIVE_TYPE.ticksAtMotor * (DRIVE_TYPE.rpm / 60)
    private val liftTicksPerSec = LIFT_TYPE.ticksAtMotor * (LIFT_TYPE.rpm / 60)
    private val collectorTicksPerSec = COLLECTOR_TYPE.ticksAtMotor * (COLLECTOR_TYPE.rpm / 60)
    private val hookTicksPerSec = HOOK_TYPE.ticksAtMotor * (HOOK_TYPE.rpm / 60)

    // Slide targets
    private var leftLiftTarget = 0
    private var rightLiftTarget = 0

    // Hook target
    private var hookTarget = 0

    // Motor types
    private enum class MotorType(val ticksAtMotor: Double, val rpm: Int) {
        CORE_HEX(288.0, 125),
        HD_HEX(28.0, 6000);
    }

    // Drive types
    private enum class DriveMode {
        JOYSTICK,
        TANK;
    }

    // Servo State Enums
    private enum class FilterState {
        NONE,
        DOWNWARDS,
        CENTER;
    }

    private enum class StorageState {
        NONE,
        OPEN,
        CLOSED;
    }

    // Servo State Variables
    private var filterState = FilterState.CENTER
    private var lastFilterState = FilterState.NONE
    private var storageState = StorageState.CLOSED
    private var lastStorageState = StorageState.NONE

    override fun runOpMode() {
        // Hardware Initialization
        right = hardwareMap.get(DcMotorEx::class.java, "right")
        left = hardwareMap.get(DcMotorEx::class.java, "left")
        rightLift = hardwareMap.get(DcMotorEx::class.java, "rightLift")
        leftLift = hardwareMap.get(DcMotorEx::class.java, "leftLift")
        collector = hardwareMap.get(DcMotorEx::class.java, "collector")
        hook = hardwareMap.get(DcMotorEx::class.java, "hook")

        filterServoRight = hardwareMap.get(Servo::class.java, "filterRight")
        filterServoLeft = hardwareMap.get(Servo::class.java, "filterLeft")
        storageServoRight = hardwareMap.get(Servo::class.java, "storageRight")
        storageServoLeft = hardwareMap.get(Servo::class.java, "storageLeft")

        // Reverse the motors
        collector.direction = DcMotorSimple.Direction.REVERSE
        right.direction = DcMotorSimple.Direction.REVERSE
        leftLift.direction = DcMotorSimple.Direction.REVERSE

        // Brake drivetrain when control released
        right.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        left.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        val initTime = System.currentTimeMillis()
        waitForStart()
        val startTime = System.currentTimeMillis()

        // Driver 1 Thread
        thread(name = "DRIVER1", start = true) {
            var collectorStartTime = 0L
            var collectorStallTime = 0L
            while (opModeIsActive()) {
                // Chassis movement
                if(DRIVE_MODE == DriveMode.JOYSTICK) {
                    left.power = -gamepad1.left_stick_y + gamepad1.left_stick_x * DRIVE_MLT
                    right.power = -gamepad1.left_stick_y - gamepad1.left_stick_x * DRIVE_MLT
                } else if(DRIVE_MODE == DriveMode.TANK) {
                    left.power = -gamepad1.left_stick_y * DRIVE_MLT
                    right.power = -gamepad1.right_stick_y * DRIVE_MLT
                }

                // Collector toggle
                if(gamepad2.b && !collectorLock) collectorOn = !collectorOn
                collectorLock = gamepad2.b

                val collectorPower = COLLECTOR_MLT

                // Collector stall detection
                if(collector.power == 0.0 && collectorOn) collectorStartTime = System.currentTimeMillis()
                collectorStartTime = maxOf(collectorStallTime + COLLECTOR_STALL_RELEASE_TIME, collectorStartTime)

                if(collector.velocity < collectorPower * 0.2 && System.currentTimeMillis() - collectorStartTime > COLLECTOR_STALL_THRESHOLD) collectorStallTime = System.currentTimeMillis()

                collector.power = if(!collectorOn) 0.0
                else if(System.currentTimeMillis() - collectorStallTime < COLLECTOR_STALL_RELEASE_TIME) -collectorPower
                else collectorPower
            }
        }

        // Driver 2 Thread
        thread(name = "DRIVER2", start = true) {
            while (opModeIsActive()) {
                // Lift (slide) movement
                val liftPower = -gamepad2.left_stick_y * LIFT_MLT
                if(gamepad2.left_stick_y != 0.0f) for (motor in listOf(rightLift, leftLift)) {
                    if(motor == leftLift) leftLiftTarget = 0
                    else rightLiftTarget = 0
                    motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                    motor.power = liftPower
                } else for (motor in listOf(rightLift, leftLift)) {
                    if(motor == leftLift && leftLiftTarget == 0) leftLiftTarget = motor.currentPosition
                    else if(motor == rightLift && rightLiftTarget == 0) rightLiftTarget = motor.currentPosition

                    motor.targetPosition = if(motor == leftLift) leftLiftTarget
                    else rightLiftTarget

                    motor.mode = DcMotor.RunMode.RUN_TO_POSITION
                    motor.power = HOLD_POWER
                }

                // Lift -> hook "master -> slave" relationship
                if(liftPower < 0) hook.power = liftPower
                else if(liftPower == 0.0) {
                    if(hookTarget == 0) hookTarget = hook.currentPosition
                    hook.targetPosition = hookTarget
                    hook.mode = DcMotor.RunMode.RUN_TO_POSITION
                    hook.power = HOLD_POWER
                } else {
                    hookTarget = 0
                    hook.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                    hook.power = 0.0
                }

                // Servo control
                if(gamepad2.dpad_down) storageState = StorageState.CLOSED
                else if(gamepad2.dpad_up) storageState = StorageState.OPEN

                if(gamepad2.a) filterState = FilterState.DOWNWARDS
                else if(gamepad2.x) filterState = FilterState.CENTER

                if(lastFilterState != filterState) {
                    when(filterState) {
                        FilterState.DOWNWARDS -> {
                            filterServoRight.position = FILTER_DOWNWARDS[0]
                            filterServoLeft.position = FILTER_DOWNWARDS[1]
                        }
                        FilterState.CENTER -> {
                            filterServoRight.position = FILTER_CENTER[0]
                            filterServoLeft.position = FILTER_CENTER[1]
                        }
                        else -> {}
                    }

                    lastFilterState = filterState
                }

                if(lastStorageState != storageState) {
                    when(storageState) {
                        StorageState.OPEN -> {
                            storageServoRight.position = STORAGE_OPEN[0]
                            storageServoLeft.position = STORAGE_OPEN[1]
                        }
                        StorageState.CLOSED -> {
                            storageServoRight.position = STORAGE_CLOSED[0]
                            storageServoLeft.position = STORAGE_CLOSED[1]
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
            telemetry.addData("Mode", DRIVE_MODE)
            telemetry.addData("Left Power", right.velocity / driveTicksPerSec)
            telemetry.addData("Right Power", left.velocity / driveTicksPerSec)
            telemetry.addData("", "")

            telemetry.addData("--- HOOK ---", "")
            if(leftLift.power < 0) telemetry.addData("Power", hook.velocity / hookTicksPerSec)
            else telemetry.addData("Target", hookTarget)

            telemetry.addData("--- Collector ---", "")
            telemetry.addData("On", collectorOn)
            telemetry.addData("Lock", collectorLock)
            telemetry.addData("Power", collector.velocity / collectorTicksPerSec)
            telemetry.addData("Stall check", collector.velocity < collectorTicksPerSec * 0.2)
            telemetry.addData("", "")

            telemetry.addData("--- SLIDES ---", "")
            telemetry.addData("Mode", rightLift.mode)
            if(rightLift.mode == DcMotor.RunMode.RUN_TO_POSITION) {
                telemetry.addData("Left target", rightLift.targetPosition)
                telemetry.addData("Right target", leftLift.targetPosition)
            } else telemetry.addData("Power", rightLift.velocity / liftTicksPerSec)
            telemetry.addData("", "")

            telemetry.addData("--- SERVOS ---", "")
            telemetry.addData("Storage state", storageState)
            telemetry.addData("Left Storage Servo", storageServoRight.position)
            telemetry.addData("Right Storage Servo", storageServoLeft.position)
            telemetry.addData("Filter state", filterState)
            telemetry.addData("Left Filter Servo", filterServoRight.position)
            telemetry.addData("Right Filter Servo", filterServoLeft.position)
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