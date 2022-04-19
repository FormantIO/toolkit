import "./App.css";
import { Button, Input, Typography } from "@alenjdev/ui-sdk";
import { useState, useEffect } from "react";
import { Authentication } from "@formant/data-sdk";

function App() {
  const [fileSize, setFileSize] = useState(0);
  const [fileName, setFileName] = useState("Nothing");
  const [adminAPI] = useState("https://api.formant.io/v1/admin");

  useEffect(() => {
    getIn();
  }, []);

  const getIn = async () => {
    if (await Authentication.waitTilAuthenticated())
      console.log(Authentication.token);
  };

  const begin_upload = async () => {
    const response = await fetch(`${adminAPI}/files/begin-upload`, {
      method: "POST",
      body: JSON.stringify({ fileName: "python.pyww", fileSize: 300 }),
      headers: {
        "Content-Type": "application/json",
        Authorization:
          "Bearer " +
          "eyJraWQiOiJRWFY0Rk5HenlpdGh3YzBxeWNzUzZGNVE5ZWJCMHY4dTBURGd4YWFQNjNNPSIsImFsZyI6IlJTMjU2In0.eyJjdXN0b206dGFncyI6Int9Iiwic3ViIjoiZjhiNzc1NDYtYmRhNC00NGQ5LTlkMjEtMWMzNTMzOTQxOTg3IiwiY29nbml0bzpncm91cHMiOlsiYWRtaW4iXSwiZW1haWxfdmVyaWZpZWQiOnRydWUsImN1c3RvbTpvcmdhbml6YXRpb25faWQiOiIwZDI5ZjY1Ni1jYzFjLTRiOWUtYmFhZC0xOTljZmExZmNjZWQiLCJpc3MiOiJodHRwczpcL1wvY29nbml0by1pZHAudXMtd2VzdC0yLmFtYXpvbmF3cy5jb21cL3VzLXdlc3QtMl91OHZBQXk0SFQiLCJjb2duaXRvOnVzZXJuYW1lIjoiZjhiNzc1NDYtYmRhNC00NGQ5LTlkMjEtMWMzNTMzOTQxOTg3IiwiYXVkIjoiMmM0dmc1OGpmNW80bGVub25pOGZuaHF2cG0iLCJldmVudF9pZCI6IjQyMWNiNWI3LTNhMWYtNGU0Yi05NGIyLTY0YWQzYTRmZWY5YyIsInRva2VuX3VzZSI6ImlkIiwiYXV0aF90aW1lIjoxNjQ5MzcxNzMwLCJleHAiOjE2NDk0MzgzODUsImlhdCI6MTY0OTQzNDc4NSwiZW1haWwiOiJhbGVuK3Byb2QrZGVtb3NAZm9ybWFudC5pbyJ9.l7Sc3XxjcS_JLDyfTADgIwAsBAvmYY-xzlvFuPAPGeDVYkao26uEDUh1wveZRKxmif31TviPwGrw5PG9vrxOY2S6F-sSqUt_c6gyspyGVGylc4Oe1tAmlFSPT2yfJ_1g3F3lE16gygkJOU8GE7A0oYTAJcKK26R6kijuy64Io4jJbSw0CTOX3iQ8DYVe4NjdJQ_ggFpKS6T6FFHnH32YLgVnmJjoW33U1lAjk2myj1XkTwMiHBR_bL2D-dxyt1hLLA6xvthaJpDrkEI3KxlCgIgavb2fSfQOQTboPnOklw0gDxaWvFvZxQp9CEe7QACVIrmiTfQ_BK7E9sJa-HEz3A",
      },
    });

    const jsonResponse = await response.json();
  };

  const complete_upload = async () => {
    const response = await fetch(`${adminAPI}/files/complete-upload`, {
      method: "POST",
      body: JSON.stringify({ fileName: "python.pyww", fileSize: 300 }),
      headers: {
        "Content-Type": "application/json",
        Authorization:
          "Bearer " +
          "eyJraWQiOiJRWFY0Rk5HenlpdGh3YzBxeWNzUzZGNVE5ZWJCMHY4dTBURGd4YWFQNjNNPSIsImFsZyI6IlJTMjU2In0.eyJjdXN0b206dGFncyI6Int9Iiwic3ViIjoiZjhiNzc1NDYtYmRhNC00NGQ5LTlkMjEtMWMzNTMzOTQxOTg3IiwiY29nbml0bzpncm91cHMiOlsiYWRtaW4iXSwiZW1haWxfdmVyaWZpZWQiOnRydWUsImN1c3RvbTpvcmdhbml6YXRpb25faWQiOiIwZDI5ZjY1Ni1jYzFjLTRiOWUtYmFhZC0xOTljZmExZmNjZWQiLCJpc3MiOiJodHRwczpcL1wvY29nbml0by1pZHAudXMtd2VzdC0yLmFtYXpvbmF3cy5jb21cL3VzLXdlc3QtMl91OHZBQXk0SFQiLCJjb2duaXRvOnVzZXJuYW1lIjoiZjhiNzc1NDYtYmRhNC00NGQ5LTlkMjEtMWMzNTMzOTQxOTg3IiwiYXVkIjoiMmM0dmc1OGpmNW80bGVub25pOGZuaHF2cG0iLCJldmVudF9pZCI6IjQyMWNiNWI3LTNhMWYtNGU0Yi05NGIyLTY0YWQzYTRmZWY5YyIsInRva2VuX3VzZSI6ImlkIiwiYXV0aF90aW1lIjoxNjQ5MzcxNzMwLCJleHAiOjE2NDk0MzgzODUsImlhdCI6MTY0OTQzNDc4NSwiZW1haWwiOiJhbGVuK3Byb2QrZGVtb3NAZm9ybWFudC5pbyJ9.l7Sc3XxjcS_JLDyfTADgIwAsBAvmYY-xzlvFuPAPGeDVYkao26uEDUh1wveZRKxmif31TviPwGrw5PG9vrxOY2S6F-sSqUt_c6gyspyGVGylc4Oe1tAmlFSPT2yfJ_1g3F3lE16gygkJOU8GE7A0oYTAJcKK26R6kijuy64Io4jJbSw0CTOX3iQ8DYVe4NjdJQ_ggFpKS6T6FFHnH32YLgVnmJjoW33U1lAjk2myj1XkTwMiHBR_bL2D-dxyt1hLLA6xvthaJpDrkEI3KxlCgIgavb2fSfQOQTboPnOklw0gDxaWvFvZxQp9CEe7QACVIrmiTfQ_BK7E9sJa-HEz3A",
      },
    });

  }

  return (
    <div className="App">
      <div className="container">
        <Input type="text" label="Stream name" onChange={() => {}} />

        <div className="btn-container">
          <input
            onChange={(ev) => {
              if (ev.target.files === null) {
                setFileName("none");
                setFileSize(0);
                return;
              }
              const uploadFile = ev.target.files[0];
              if (uploadFile.size === 0) return;

              setFileName(uploadFile.name);
              setFileSize(uploadFile.size);
              // // console.log(typeof uploadFile.name, typeof uploadFile.size);
              // begin_upload(uploadFile.name, uploadFile.size);
            }}
            hidden
            type="file"
            id="real-file"
          />
          <Button
            onClick={() => {
              let btn = document.getElementById("real-file");
              btn?.click();
            }}
            type="primary"
            size="large"
          >
            UPLOAD
          </Button>
          <Button onClick={() => begin_upload()} type="primary" size="large">
            DONE
          </Button>
          <Typography type="h3">{fileName}</Typography>
        </div>
      </div>
    </div>
  );
}

export default App;
