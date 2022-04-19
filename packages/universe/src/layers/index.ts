import { DataLayer } from "./DataLayer";
import { LabelLayer } from "./LabelLayer";
import { LayerRegistry } from "./LayerRegistry";
import { TransformLayer } from "./TransformLayer";
import { GroundLayer } from "./GroundLayer";

LayerRegistry.register(DataLayer);
LayerRegistry.register(LabelLayer);
LayerRegistry.register(TransformLayer);
LayerRegistry.register(GroundLayer);

export type LayerType = string;
