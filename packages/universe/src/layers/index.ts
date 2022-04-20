import { DataLayer } from "./DataLayer";
import { LabelLayer } from "./LabelLayer";
import { LayerRegistry } from "./LayerRegistry";
import { TransformLayer } from "./TransformLayer";
import { GroundLayer } from "./GroundLayer";
import { ImageLayer } from "./ImageLayer";
import { GltfLayer } from "./GltfLayer";

LayerRegistry.register(DataLayer);
LayerRegistry.register(LabelLayer);
LayerRegistry.register(TransformLayer);
LayerRegistry.register(GroundLayer);
LayerRegistry.register(ImageLayer);
LayerRegistry.register(GltfLayer);
LayerRegistry.register(GroundLayer);

export type LayerType = string;
