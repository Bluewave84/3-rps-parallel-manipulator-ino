import express, { Express, Request, Response } from 'express';
import cors from 'cors';
import path from 'path';
import { RobotKinematics } from './robotKinematics';
import { RobotController } from './controller';

// Types
interface UpdateRequestBody {
    slider_theta: number;
    slider_phi: number;
    slider_h: number;
}

interface Point3D {
    x: number;
    y: number;
    z: number;
}

interface RobotResponse {
    alpha: number;
    beta: number;
    gamma: number;
    h: number;
    theta1: number;
    theta2: number;
    theta3: number;
    A_points: Point3D[];
    B_points: Point3D[];
    C_points: Point3D[];
    line_A: Point3D[];
    line_B: Point3D[];
    line_C1: Point3D[];
    line_C2: Point3D[];
    line_C3: Point3D[];
    lp: number;
    l1: number;
    l2: number;
    lb: number;
    minh: number;
    maxh: number;
    max_theta: number;
}

class RobotApp {
    private readonly app: Express;
    private readonly robot: RobotKinematics;
    private readonly robotController: RobotController;

    constructor() {
        this.app = express();
        this.robot = new RobotKinematics();
        this.robotController = new RobotController(this.robot);
        
        this.initializeRobot();
        this.setupMiddleware();
        this.setupRoutes();
    }

    private initializeRobot(): void {
        // Hard-coded parameters
        this.robot.lp = 7.125;
        this.robot.l1 = 6.2;
        this.robot.l2 = 4.5;
        this.robot.lb = 4.0;

        this.robot.maxh = this.computeMaxh(
            this.robot.l1,
            this.robot.l2,
            this.robot.lp,
            this.robot.lb
        ) - 0.2;

        this.robot.minh = this.computeMinh(
            this.robot.l1,
            this.robot.l2,
            this.robot.lp,
            this.robot.lb
        ) + 0.45;
    }

    private computeMaxh(l1: number, l2: number, lp: number, lb: number): number {
        return Math.sqrt(((l1 + l2) ** 2) - ((lp - lb) ** 2));
    }

    private computeMinh(l1: number, l2: number, lp: number, lb: number): number {
        if (l1 > l2) {
            return Math.sqrt((l1 ** 2) - ((lb + l2 - lp) ** 2));
        } else if (l2 > l1) {
            return Math.sqrt(((l2 - l1) ** 2) - ((lp - lb) ** 2));
        }
        return 0;
    }

    private setupMiddleware(): void {
        this.app.use(cors());
        this.app.use(express.json());
        this.app.use(express.static(path.join(__dirname, 'public')));
    }

    private setupRoutes(): void {
        this.app.get('/', (req: Request, res: Response) => {
            res.sendFile(path.join(__dirname, 'templates', 'index.html'));
        });

        this.app.post('/update', this.handleUpdate.bind(this));
    }

    private handleUpdate(req: Request<{}, {}, UpdateRequestBody>, res: Response): void {
        try {
            const {
                slider_theta = 0,
                slider_phi = 0,
                slider_h = 814
            } = req.body;

            const theta_deg = slider_theta / 100.0;  // degrees
            const phi_deg = slider_phi / 100.0;     // degrees
            const h = slider_h / 100.0;

            let alpha = 0.0;
            let beta = 0.0;
            let gamma = 1.0;
            let max_theta_for_h = 10.0;  // fallback

            const theta_rad = this.toRadians(theta_deg);
            const phi_rad = this.toRadians(phi_deg);
            
            alpha = Math.sin(theta_rad) * Math.cos(phi_rad);
            beta = Math.sin(theta_rad) * Math.sin(phi_rad);
            gamma = Math.cos(theta_rad);

            max_theta_for_h = this.robot.maxTheta(h);
            this.robot.solveInverseKinematicsVector(alpha, beta, gamma, h);

            // Set motor angles
            this.robotController.setMotorAngles(
                this.toDegrees(Math.PI * 0.5 - this.robot.theta1),
                this.toDegrees(Math.PI * 0.5 - this.robot.theta2),
                this.toDegrees(Math.PI * 0.5 - this.robot.theta3)
            );

            const response: RobotResponse = {
                alpha,
                beta,
                gamma,
                h,
                theta1: this.robot.theta1 ?? 0.0,
                theta2: this.robot.theta2 ?? 0.0,
                theta3: this.robot.theta3 ?? 0.0,
                A_points: [this.robot.A1, this.robot.A2, this.robot.A3],
                B_points: [this.robot.B1, this.robot.B2, this.robot.B3],
                C_points: [this.robot.C1, this.robot.C2, this.robot.C3],
                line_A: [
                    this.robot.A1, this.robot.A2,
                    this.robot.A2, this.robot.A3,
                    this.robot.A3, this.robot.A1
                ],
                line_B: [
                    this.robot.B1, this.robot.B2,
                    this.robot.B2, this.robot.B3,
                    this.robot.B3, this.robot.B1
                ],
                line_C1: [this.robot.A1, this.robot.C1, this.robot.C1, this.robot.B1],
                line_C2: [this.robot.A2, this.robot.C2, this.robot.C2, this.robot.B2],
                line_C3: [this.robot.A3, this.robot.C3, this.robot.C3, this.robot.B3],
                lp: this.robot.lp,
                l1: this.robot.l1,
                l2: this.robot.l2,
                lb: this.robot.lb,
                minh: this.robot.minh,
                maxh: this.robot.maxh,
                max_theta: max_theta_for_h
            };

            res.json(response);
        } catch (error) {
            console.error('Error in update:', error);
            res.status(500).json({ error: 'Internal server error' });
        }
    }

    private toRadians(degrees: number): number {
        return degrees * (Math.PI / 180);
    }

    private toDegrees(radians: number): number {
        return radians * (180 / Math.PI);
    }

    public start(port: number = 8000): void {
        this.app.listen(port, '0.0.0.0', () => {
            console.log(`Server running at http://0.0.0.0:${port}`);
        });
    }
}

// Start the application
if (require.main === module) {
    const app = new RobotApp();
    app.start(8000);
}

export { RobotApp };
