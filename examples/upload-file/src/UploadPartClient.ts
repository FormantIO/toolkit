export class UploadPartClient {
  public async upload(partUrl: string, blob: Blob): Promise<string> {
    const result = await fetch(partUrl, {
      method: "PUT",
      body: blob,
    });
    const eTag = result.headers.get("etag");
    if (!eTag) {
      throw new Error(`Invalid ETag from upload part response: ${eTag}`);
    }
    return eTag;
  }
}
