const formantApiUrl = "https://api.formant.io";

export type DataSdkInitOptions = {
  adminApi: string;
  queryApi: string;
  ingestionApi: string;
  signalingApi: string;
};

function stripTrailingSlash(url: string): string {
  return url.replace(/\/+$/, "");
}

export class DataSdk {
  private constructor() {}

  static adminApi = `${formantApiUrl}/v1/admin`;
  static queryApi = `${formantApiUrl}/v1/queries`;
  static ingestionApi = `${formantApiUrl}/v1/ingest`;
  static signalingApi = formantApiUrl;

  static init(options: DataSdkInitOptions): void {
    DataSdk.adminApi = stripTrailingSlash(options.adminApi);
    DataSdk.queryApi = stripTrailingSlash(options.queryApi);
    DataSdk.ingestionApi = stripTrailingSlash(options.ingestionApi);
    DataSdk.signalingApi = stripTrailingSlash(options.signalingApi);
  }
}
