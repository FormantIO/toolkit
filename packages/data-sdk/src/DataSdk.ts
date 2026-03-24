const formantApiUrl = "https://api.formant.io";

export type DataSdkInitOptions = {
  adminApi: string;
  queryApi: string;
  ingestionApi: string;
  signalingApi: string;
};

export class DataSdk {
  private constructor() {}

  static adminApi = `${formantApiUrl}/v1/admin`;
  static queryApi = `${formantApiUrl}/v1/queries`;
  static ingestionApi = `${formantApiUrl}/v1/ingest`;
  static signalingApi = formantApiUrl;

  static init(options: DataSdkInitOptions): void {
    DataSdk.adminApi = options.adminApi;
    DataSdk.queryApi = options.queryApi;
    DataSdk.ingestionApi = options.ingestionApi;
    DataSdk.signalingApi = options.signalingApi;
  }
}
