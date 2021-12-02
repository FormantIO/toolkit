function generateBinaryId() {
    // attach binary ID as 16-byte header in binary messages
    const id = new Uint8Array(16);
    for (let i = 0; i < id.length; i++) {
        id[i] = Math.floor(Math.random() * 256);
    }
    return id;
}

console.log(generateBinaryId().toString());
console.log(generateBinaryId().toString().length);
