import { VideoMimeType } from "./VideoMimeType";

export interface IVideo {
    url: string;
    size?: number;
    duration: number;
    mimeType: VideoMimeType;
}
