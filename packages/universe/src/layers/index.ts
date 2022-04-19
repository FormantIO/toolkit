import { DataLayer } from "./DataLayer";
import { LabelLayer } from "./LabelLayer";
import { LayerRegistry } from "./LayerRegistry";
import { TransformLayer } from "./TransformLayer";

LayerRegistry.register(DataLayer);
LayerRegistry.register(LabelLayer);
LayerRegistry.register(TransformLayer);

export type LayerType = string;
