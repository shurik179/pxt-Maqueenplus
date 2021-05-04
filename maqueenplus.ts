/**
 * @file pxt-DFRobot_Maqueenplus/maqueenplus.ts
 * @brief DFRobot's maqueenplus makecode library.
 * @n [Get the module here](https://www.dfrobot.com/product-2026.html)
 * @n Maqueen plus is a  STEM educational robot for micro:bit. Optimized with better power management and larger capacity power supply, it can be perfectly compatible with Huskylens AI Vision Sensor.
 *
 * @copyright    [DFRobot](http://www.dfrobot.com), 2021
 * @copyright    MIT
 *
 * @author [email](jie.tang@dfrobot.com)
 * @date  2021-05-02
 */

let maqueencb: Action
let maqueenmycb: Action
let maqueene = "1"
let maqueenparam = 0
let alreadyInit = 0
let IrPressEvent = 0
let TICKS_TO_MM = 1.6

enum PIN {
    P0 = 3,
    P1 = 2,
    P2 = 1,
    P8 = 18,
    //P9 = 10,
    P12 = 20,
    P13 = 23,
    P14 = 22,
    P15 = 21,
};

enum Motors {
    //% block="left"
    M1 = 1,
    //% block="right"
    M2 = 2,
}

enum Dir {
    //% block="rotate forward"
    CW = 1,
    //% block="backward"
    CCW = 2
}

enum Servos {
    //% block="S1"
    S1 = 1,
    //% block="S2"
    S2 = 2,
    //% block="S3"
    S3 = 3
}

enum RGBLight {
    //%block="RGB_L"
    LEFT = 1,
    //%block="RGB_R"
    RIGHT = 2,
    //%block="ALL"
    ALL = 3
}

enum LineSensor {
    //% block="L1"
    L3 = 0,
    //%block="L2"
    L2 = 1,
    //%block="L3"
    L1 = 2,
    //%block="R1"
    R1 = 3,
    //%block="R2"
    R2 = 4,
    //%block="R3"
    R3 = 5
}

enum Sonicunit {
    //% block="cm"
    Centimeters
}

enum PID {
    //%block="OFF"
    OFF = 0,
    //%block="ON"
    ON = 1
}

enum Color {
    //%block="Red"
    RED = 1,
    //%block="Green"
    GREEN = 2,
    //%block="Blue"
    BLUE = 4,
    //%block="Yellow"
    YELLOW = 3,
    //%block="Violet"
    PINK = 5,
    //%block="Cyan"
    CYAN = 6,
    //%block="White"
    WHITE = 7,
    //%block="OFF"
    OFF = 8

}

//% weight=100  color=#00A654   block="MaqueenPlus" icon="\uf013"
namespace MaqueenPlus {
    let irstate:number;
    let state:number;
    let speedLeft:number;
    let speedRight: number;
    let _encoderL:number;
    let _encoderR: number;
    let lineRaw:number[]=[0,0,0,0,0,0];
    export class Packeta {
        public mye: string;
        public myparam: number;
    }

    /**
     *  Init I2C until success
     */
    //% weight=100
    //%block="initialize via I2C until success"
    export function I2CInit():void{
        let Version_v = 0;
        pins.i2cWriteNumber(0x10, 0x32, NumberFormat.Int8LE);
        Version_v = pins.i2cReadNumber(0x10, NumberFormat.Int8LE);
        while (Version_v==0){
            basic.showLeds(`
                # . . . #
                . # . # .
                . . # . .
                . # . # .
                # . . . #
                `, 10)
            basic.pause(500)
            basic.clearScreen()
            pins.i2cWriteNumber(0x10, 0x32, NumberFormat.Int8LE);
            Version_v = pins.i2cReadNumber(0x10, NumberFormat.Int8LE);
        }
        basic.showLeds(`
                . . . . .
                . . . . #
                . . . # .
                # . # . .
                . # . . .
                `, 10)
        basic.pause(500)
        basic.clearScreen()
    }

    /**
     * PID control module
     */
    //% weight=90
    //%block="PID switch|%pid"
    export function PID(pid: PID): void {
        let buf = pins.createBuffer(2);
        buf[0] = 0x0A;
        buf[1] = pid;
        pins.i2cWriteBuffer(0x10, buf);
    }
    /**
     * Motor control module
     */
    //% weight=80
    //% block="set speed |left|%speed_L|right|%speed_R"
    //% speed_L.min=-100 speed_L.max=100 speed_R.min=-100 speed_R.max=100
    export function setMotors(speed_L: number, speed_R:number): void {
        let _dir_L:number;
        let _dir_R:number;
        let _speed_L:number;
        let _speed_R:number;
        if (speed_L<0) {
            _dir_L=2;
            _speed_L=(-speed_L*255)/100;
        }  else {
            _dir_L=1;
            _speed_L=(speed_L*255)/100;
        }
        if (_speed_L>255) _speed_L=255;
        if (speed_R<0) {
            _dir_R=2;
            _speed_R=(-speed_R*255)/100;
        }  else {
            _dir_R=1;
            _speed_R=(speed_R*255)/100;
        }
        if (_speed_R>255) _speed_R=255;

        let buf = pins.createBuffer(5)
        buf[0] = 0x00;
        buf[1] = _dir_L;
        buf[2] = _speed_L;
        buf[3] = _dir_R;
        buf[4] = _speed_R;
        pins.i2cWriteBuffer(0x10, buf)

    }
    /**
     * Motor stop module
     */
    //% weight=75
    //% block="Stop motors"
    export function stopMotors(): void {
        let buf = pins.createBuffer(5);
        buf[0] = 0x00;
        buf[1] = 0;
        buf[2] = 0;
        buf[3] = 0;
        buf[4] = 0;
        pins.i2cWriteBuffer(0x10, buf);
    }


    /**
     * Read motor speed
     */
    //% weight=65
    //%block="Get and save speed"
    export function getSpeed(): void {
        let _dir:number;
        pins.i2cWriteNumber(0x10, 0, NumberFormat.Int8LE);
        let speed_x = pins.i2cReadBuffer(0x10, 4);
        _dir = speed_x[0];
        if (_dir == 1) {
            speedLeft=speed_x[1];
        } else {
            speedLeft=-speed_x[1];
        }
        _dir = speed_x[2];
        if (_dir == 1) {
            speedRight = speed_x[3];
        } else {
            speedRight = -speed_x[3];
        }
    }
    /**
     * Access recently read motor speed
     */
    //% weight = 66
    //%block ="motor|%index speed"
    export function   readSpeed(index: Motors): number {
        if (index == 1) {
            return speedLeft;
        }
        return speedRight;
    }



    /**
     * Servo control module
     */
    //% weight=40
    //% block="servo|%index|angle|%angle"
    //% angle.min=0  angle.max=180
    export function setServo(index: Servos, angle: number): void {
        let buf = pins.createBuffer(2)
        switch (index) {
            case 1:
                buf[0] = 0x14;
                buf[1] = angle;
                pins.i2cWriteBuffer(0x10, buf);
                break;
            case 2:
                buf[0] = 0x15;
                buf[1] = angle;
                pins.i2cWriteBuffer(0x10, buf);
                break;
            default:
                buf[0] = 0x16;
                buf[1] = angle;
                pins.i2cWriteBuffer(0x10, buf);
                break;
        }
    }

    /**
     * Control the color of RGB LED
     */
    //% weight=50
    //% block="set |%rgbshow color|%color"
    export function setRGBLight(rgbshow: RGBLight, color: Color): void {

        if (rgbshow == 1) {
            let buf = pins.createBuffer(2)
            buf[0] = 0x0B;
            buf[1] = color;
            pins.i2cWriteBuffer(0x10, buf);
        } if (rgbshow == 2) {
            let buf = pins.createBuffer(2)
            buf[0] = 0x0C;
            buf[1] = color;
            pins.i2cWriteBuffer(0x10, buf);
        } if (rgbshow == 3) {
            let buf = pins.createBuffer(3)
            buf[0] = 0x0B;
            buf[1] = color;
            buf[2] = color;
            pins.i2cWriteBuffer(0x10, buf);
        }

    }

    /**
     * Read grayscale value of line-tracking sensor
     */
    //% weight=55
    //% block="read line-tracking sensor|%patrol grayscale "
    export function getLineSensors(): void {
        pins.i2cWriteNumber(0x10, 0x1E, NumberFormat.Int8LE);
        let data = pins.i2cReadBuffer(0x10, 12);
        for (let i=0; i<6; i++) {
            lineRaw[i] = data[2*i+1] | data[2*i] << 8;
        }

    }
    /**
     * Access recently read line sensor raw values
     */
    //% weight = 56
    //%block =" line sensor |%index raw value"
    export function   LineSensorRaw(index: LineSensor): number {
        return lineRaw[index];
    }


    /**
     * get the revolutions of wheel
     */
    //% weight=60
    //%block="update encoder readings"
    export function updateEncoders():void {
        let distance:number;
        pins.i2cWriteNumber(0x10, 4, NumberFormat.Int8LE);
        let speed_x = pins.i2cReadBuffer(0x10, 4);
        _encoderL = (speed_x[0]<<8|speed_x[1]);
        _encoderR = (speed_x[2]<<8|speed_x[3]);
    }

    /**
     * get the revolutions of wheel
     */
    //% weight=60
    //%block="encoder reading for motor|%motor "
    export function encoder(motor: Motors):void {
        if (motor == 1) return (_encoderL)
        return (_encoderR)
    }

    /**
     * clear the motor encoders (rotation counters)
     */
    //% weight=60
    //%block="reset the encoder count"
    export function resetEncoders():void{
                let buf3 = pins.createBuffer(4);
                buf3[0] = 0x04;
                buf3[1] = 0;
                buf3[2] = 0;
                buf3[3] = 0;
                pins.i2cWriteBuffer(0x10, buf3);
    }


    /**
     * Get product information
     */
    //% weight=5
    //%block="get product information"
    export function readVersion(): string {
        pins.i2cWriteNumber(0x10, 0x32, NumberFormat.Int8LE);
        let Version_v = pins.i2cReadNumber(0x10, NumberFormat.Int8LE);
        pins.i2cWriteNumber(0x10, 0x33, NumberFormat.Int8LE);
        let Version_y = pins.i2cReadBuffer(0x10, Version_v);
        let Version_x = Version_y.toString();
        return Version_x;
    }
    /**
     * Read the distance value the ultrasound returns
     */
    //% weight=20
    //%block="read ultrasonic sensor TRIG %T ECHO %E Company:CM"
    export function ultraSonic(T: PIN, E: PIN): number {
        let maxCmDistance = 500;
        let _T;
        let _E;
        switch (T) {
            case PIN.P0: _T = DigitalPin.P0; break;
            case PIN.P1: _T = DigitalPin.P1; break;
            case PIN.P2: _T = DigitalPin.P2; break;
            case PIN.P8: _T = DigitalPin.P8; break;
            case PIN.P12: _T = DigitalPin.P12; break;
            // case PIN.P10: _T = DigitalPin.P10; break;
            case PIN.P13: _T = DigitalPin.P13; break;
            case PIN.P14: _T = DigitalPin.P14; break;
            case PIN.P15: _T = DigitalPin.P15; break;
            default: _T = DigitalPin.P0; break;
        }

        switch (E) {
            case PIN.P0: _E = DigitalPin.P0; break;
            case PIN.P1: _E = DigitalPin.P1; break;
            case PIN.P2: _E = DigitalPin.P2; break;
            case PIN.P8: _E = DigitalPin.P8; break;
            //case PIN.P9: _E = DigitalPin.P9; break;
            case PIN.P12: _E = DigitalPin.P12; break;
            case PIN.P13: _E = DigitalPin.P13; break;
            case PIN.P14: _E = DigitalPin.P14; break;
            case PIN.P15: _E = DigitalPin.P15; break;
            default: _E = DigitalPin.P0; break;
        }

        let ultraSonic_d;
        pins.digitalWritePin(_T, 1);
        basic.pause(1)
        pins.digitalWritePin(_T, 0);
        if (pins.digitalReadPin(_E) == 0) {
            pins.digitalWritePin(_T, 1);
            // basic.pause(10);
            pins.digitalWritePin(_T, 0);
            ultraSonic_d = pins.pulseIn(_E, PulseValue.High, maxCmDistance * 58);
        } else {
            pins.digitalWritePin(_T, 0);
            // basic.pause(10);
            pins.digitalWritePin(_T, 1);
            ultraSonic_d = pins.pulseIn(_E, PulseValue.Low, maxCmDistance * 58);
        }
        let ultraSonic_x = ultraSonic_d / 39;
        if (ultraSonic_x <= 0 || ultraSonic_x > 500) {
            return 0;
        }
        return Math.round(ultraSonic_x);


    }
    /**
     *  infra-red sensor
     */
    //% advanced=true shim=maqueenIR::initIR
    function initIR(pin: Pins): void {
        return
    }
    //% advanced=true shim=maqueenIR::onPressEvent
    function onPressEvent(btn: RemoteButton, body: Action): void {
        return
    }
    //% advanced=true shim=maqueenIR::getParam
    function getParam(): number {
        return 0
    }

    function maqueenInit(): void {
        if (alreadyInit == 1) {
            return
        }
        initIR(Pins.P16);
        alreadyInit = 1;
    }

    /**
     * Run when received IR signal
     */
    //% weight=10
    //%  block="on IR received"
    export function IR_callbackUser(maqueencb: (message: number) => void) {
        maqueenInit();
        IR_callback(() => {
            const packet = new Packeta();
            packet.mye = maqueene;
            maqueenparam = getParam();
            packet.myparam = maqueenparam;
            maqueencb(packet.myparam);
        });
    }

    /**
     * Read the IR information
     */
    //% weight=15
    //%  block="read IR"
    export function IR_read(): number {
        maqueenInit();
        return getParam();
    }

    function IR_callback(a: Action): void {
        maqueencb = a;
        IrPressEvent += 1;
        onPressEvent(IrPressEvent, maqueencb);
    }

    /**
     * Read IR sensor value V2.
     */

    //% advanced=true shim=maqueenIRV2::irCode
    function irCode(): number {
        return 0;
    }

    //% weight=5
    //% group="micro:bit(v2)"
    //% block="read IR key value"
    export function IR_readV2(): number {
        return valuotokeyConversion();
    }

    //% weight=2
    //% group="micro:bit(v2)"
    //% block="on IR received"
    //% draggableParameters
    export function IR_callbackUserV2(cb: (message: number) => void) {
        state = 1;
        control.onEvent(11, 22, function() {
            cb(irstate)
        })
    }

function valuotokeyConversion():number{
    let irdata:number;
    switch(irCode()){
        case 0xff00:irdata = 0;break;
        case 0xfe01:irdata = 1;break;
        case 0xfd02:irdata = 2;break;
        case 0xfb04:irdata = 4;break;
        case 0xfa05:irdata = 5;break;
        case 0xf906:irdata = 6;break;
        case 0xf708:irdata = 8;break;
        case 0xf609:irdata = 9;break;
        case 0xf50a:irdata = 10;break;
        case 0xf30c:irdata = 12;break;
        case 0xf20d:irdata = 13;break;
        case 0xf10e:irdata = 14;break;
        case 0xef10:irdata = 16;break;
        case 0xee11:irdata = 17;break;
        case 0xed12:irdata = 18;break;
        case 0xeb14:irdata = 20;break;
        case 0xea15:irdata = 21;break;
        case 0xe916:irdata = 22;break;
        case 0xe718:irdata = 24;break;
        case 0xe619:irdata = 25;break;
        case 0xe51a:irdata = 20;break;
        default:
         irdata = -1;
    }
    return irdata;
}

    basic.forever(() => {
        if(state == 1){
             irstate = valuotokeyConversion();
        if(irstate != -1){
            control.raiseEvent(11, 22)
        }
        }

        basic.pause(50);
    })
}
