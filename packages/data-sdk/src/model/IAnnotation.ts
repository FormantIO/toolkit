import { IBaseEvent } from "./IBaseEvent";
import { IsoDate } from "./IsoDate";
import { ISpreadsheetIdRange } from "./ISpreadsheetIdRange";
import { ITaggedUsers } from "./ITaggedUsers";
import { Uuid } from "./Uuid";

export interface IAnnotation extends IBaseEvent<"annotation"> {
    editedAt?: IsoDate;
    userId: Uuid;
    annotationTemplateId: Uuid;
    taggedUsers?: ITaggedUsers | null;
    publishedTo?: ISpreadsheetIdRange;
    note?: string | null;
}

// "or null" types are to support clearing fields via HTTP PATCH
