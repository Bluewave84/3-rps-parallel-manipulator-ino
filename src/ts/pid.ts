enum ConversionType {
    Linear = "linear",
    Tanh = "tanh"
}

interface Vector2D {
    x: number;
    y: number;
}

class PIDController {
    private readonly kp: number;
    private readonly ki: number;
    private readonly kd: number;
    private readonly alpha: number;
    private readonly beta: number;
    private readonly maxTheta: number;
    private readonly magnitudeConvert: number;

    private prevOutX: number = 0;
    private prevErrX: number = 0;
    private prevOutY: number = 0;
    private prevErrY: number = 0;

    private sumErrX: number = 0;
    private sumErrY: number = 0;

    private lastTime: number | null = null;

    constructor(
        kp: number,
        ki: number,
        kd: number,
        alpha: number,
        beta: number,
        maxTheta: number,
        conversion: ConversionType = ConversionType.Linear
    ) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.alpha = alpha;  // Exponential Filter: α⋅x + (1-α)⋅x_last
        this.beta = beta;    // Coefficient for converting magnitude
        this.maxTheta = maxTheta;

        this.magnitudeConvert = conversion === ConversionType.Linear ? 1 : 0;
    }

    /**
     * Calculates PID control output in spherical coordinates
     * @param target Target position vector
     * @param current Current position vector
     * @returns Tuple of [theta, phi] in degrees
     */
    public pid(target: Vector2D, current: Vector2D): [number, number] {
        // Calculate time delta
        const newTime = performance.now() / 1000; // Convert to seconds
        const dt = this.lastTime !== null ? newTime - this.lastTime : 0.001;

        // Calculate errors
        const errX = current.x - target.x;
        const errY = current.y - target.y;
        
        // Update integral terms
        this.sumErrX += errX * dt;
        this.sumErrY += errY * dt;

        // Calculate derivative terms
        const dErrX = dt > 0 ? (errX - this.prevErrX) / dt : 0;
        const dErrY = dt > 0 ? (errY - this.prevErrY) / dt : 0;

        // Calculate PID outputs
        const pidX = this.kp * errX + this.ki * this.sumErrX + this.kd * dErrX;
        const pidY = this.kp * errY + this.ki * this.sumErrY + this.kd * dErrY;

        // Apply exponential filter
        const filteredX = this.alpha * pidX + (1 - this.alpha) * this.prevOutX;
        const filteredY = this.alpha * pidY + (1 - this.alpha) * this.prevOutY;

        // Convert to spherical coordinates
        let phi = this.toDegrees(Math.atan2(filteredY, filteredX));
        phi = phi < 0 ? phi + 360 : phi;

        const r = Math.sqrt(filteredX ** 2 + filteredY ** 2);
        const theta = this.magnitudeConvert === 1
            ? Math.min(Math.max(0, this.beta * r), this.maxTheta)
            : Math.max(0, 15 * Math.tanh(this.beta * r));

        // Update previous values
        this.prevErrX = errX;
        this.prevErrY = errY;
        this.prevOutX = filteredX;
        this.prevOutY = filteredY;
        this.lastTime = newTime;

        return [theta, phi];
    }

    private toDegrees(radians: number): number {
        return radians * (180 / Math.PI);
    }
}

/* usage example 
const controller = new PIDController(
    1.0,    // kp
    0.5,    // ki
    0.2,    // kd
    0.8,    // alpha
    0.5,    // beta
    45,     // maxTheta
    ConversionType.Linear
);

const target: Vector2D = { x: 0, y: 0 };
const current: Vector2D = { x: 1, y: 1 };
const [theta, phi] = controller.pid(target, current);
*/
