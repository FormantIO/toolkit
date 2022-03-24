import { IGoal } from "./IGoal";
import { IMap } from "./IMap";
import { IOdometry } from "./IOdometry";
import { IPath } from "./IPath";
import { IPointCloud } from "./IPointCloud";

export interface ILocalization {
    odometry: IOdometry;
    map?: IMap;
    pointClouds?: IPointCloud[];
    path?: IPath;
    goal?: IGoal;
}
