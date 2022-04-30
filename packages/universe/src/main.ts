import { DataLayer } from './layers/DataLayer';
import { LabelLayer } from './layers/LabelLayer';
import { LayerRegistry } from './layers/LayerRegistry';
import { TransformLayer } from './layers/TransformLayer';
import { GroundLayer } from './layers/GroundLayer';
import { ImageLayer } from './layers/ImageLayer';
import { GltfLayer } from './layers/GltfLayer';

export * from './Universe';
export * from './IUniverseData';
export * from './layers/LayerRegistry';
export * from './layers/UniverseLayerContent';
export * from './layers/TransformLayer';
export * from './sidebar';
export * from './SimulatedUniverseData';

LayerRegistry.register(DataLayer);
LayerRegistry.register(LabelLayer);
LayerRegistry.register(TransformLayer);
LayerRegistry.register(GroundLayer);
LayerRegistry.register(ImageLayer);
LayerRegistry.register(GltfLayer);
LayerRegistry.register(GroundLayer);

export type LayerType = string;
