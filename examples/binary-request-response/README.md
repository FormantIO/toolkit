# Binary request response

This custom view demonstrates how RequestDataChannel can be used to write applications with a request-response pattern. This particular example is for binary request data channels, which accept Uint8Array requests in javascript and can be used for protocols like protobuf. Press spacebar to make a request. If the adapter is running, a response will come in. If no adapter is running, the request will time out.

## Run locally

go to the project directory

```bash
  cd toolkit/examples/binary-request-response
```

Install dependencies

```
   npm i
```

Start the server

```bash
  npm run dev
```

### Running the adapter

1. Install the `formant` python module. If it's already installed, make sure it is up-to-date. `python3 -m pip install --upgrade formant`
2. Run `python3 adapter.py` with a Formant agent running on the same host.
